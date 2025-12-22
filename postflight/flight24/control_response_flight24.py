#!/usr/bin/env python3
"""
Control→state response plots for flight24, styled to match flight22/23.
- Sim: commands (-1..1) dashed on left, roll/pitch (rad) and speed (m/s) solid on right.
- Xiao: RC (1000–2000 µs) dashed on left, roll/pitch (rad) and speed (m/s) solid on right, per span.
Outputs: control_response_sim.png and control_response_xiao_span#.png

COORDINATE NOTE (2025-12-20):
- INAV quaternions are body->earth in NED frame
- msplink.cpp applies conjugate to convert to earth->body for GP
- Logged quaternions in xiao log are earth->body (post-conjugate)
"""
import math
import re
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
SIM_DATA = Path("/home/gmcnutt/GP/autoc/data.dat")
XIAO_LOG = Path("/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/fligt24-hb1-flight_log_2025-12-21T18-10-34.txt")

# Colors (match flight22): roll red, pitch green, throttle/speed blue
ROLL_COLOR = "tab:red"
PITCH_COLOR = "tab:green"
THR_COLOR = "tab:blue"
SPEED_COLOR = THR_COLOR


def quat_to_euler_deg(qw, qx, qy, qz) -> Tuple[float, float, float]:
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def load_sim(max_lines: int = 200000) -> pd.DataFrame:
    cols = [
        "pthstep", "Time", "Idx", "totDist", "pathX", "pathY", "pathZ",
        "X", "Y", "Z", "dr", "dp", "dy", "relVel", "roll_cmd", "pitch_cmd", "power_cmd",
        "distP", "angleP", "movEffP", "qw", "qx", "qy", "qz", "vxBody", "vyBody", "vzBody",
        "alpha", "beta", "dtheta", "dphi", "dhome", "xtrkP", "xOscP", "orientP",
    ]
    rows = []
    with SIM_DATA.open() as f:
        for i, line in enumerate(f):
            if i < 2:
                continue
            if i > max_lines:
                break
            tokens = line.strip().split()
            if len(tokens) != len(cols):
                continue
            rows.append(tokens)
    if not rows:
        return pd.DataFrame()
    raw = pd.DataFrame(rows, columns=cols)
    for c in raw.columns:
        raw[c] = pd.to_numeric(raw[c], errors="coerce")
    df = raw.dropna(subset=["qw", "qx", "qy", "qz"]).reset_index(drop=True)
    df["time_ms"] = pd.to_numeric(df["Time"], errors="coerce")
    df["throttle_cmd"] = df["power_cmd"]
    eulers = df.apply(lambda r: pd.Series(quat_to_euler_deg(r.qw, r.qx, r.qy, r.qz), index=["roll_deg", "pitch_deg", "yaw_deg"]), axis=1)
    df = pd.concat([df, eulers], axis=1)
    df["roll_rad"] = df["roll_deg"] * math.pi / 180.0
    df["pitch_rad"] = df["pitch_deg"] * math.pi / 180.0
    df["speed_mag"] = np.sqrt(df["vxBody"] ** 2 + df["vyBody"] ** 2 + df["vzBody"] ** 2)
    df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) * 0.001
    # Rates from quaternion-derived angles
    if len(df) >= 2:
        df["roll_rate"] = np.gradient(df["roll_rad"], df["time_s"], edge_order=2)
        df["pitch_rate"] = np.gradient(df["pitch_rad"], df["time_s"], edge_order=2)
    else:
        df["roll_rate"] = np.nan
        df["pitch_rate"] = np.nan
    return df


class Span:
    def __init__(self, start_ms: int, end_ms: int, label: str):
        self.start_ms = start_ms
        self.end_ms = end_ms
        self.label = label
        self.states: List[dict] = []
        self.outputs: List[dict] = []
        self.inputs: List[dict] = []  # GP sensor inputs (alpha, beta, dtheta, dphi, dhome)


def parse_xiao_spans() -> List[Span]:
    spans: List[Span] = []
    current = None
    control_re = re.compile(r"^#\d+\s+\d+\s+(\d+)\s+[iwe]\s+GP Control: (Switch enabled|Switch disabled)")
    state_re = re.compile(
        r"^#\d+\s+(\d+)\s+(\d+)\s+[iwe]\s+GP State: .*vel=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\].*quat=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\]"
    )
    output_re = re.compile(r"^#\d+\s+(\d+)\s+(\d+)\s+[iwe]\s+GP Output: rc=\[(\d+),(\d+),(\d+)\]")
    input_re = re.compile(
        r"^#\d+\s+(\d+)\s+(\d+)\s+[iwe]\s+GP Input:.*alpha\[0\]=([-0-9\.]+)\s+beta\[0\]=([-0-9\.]+)\s+dtheta\[0\]=([-0-9\.]+)\s+dphi\[0\]=([-0-9\.]+)\s+dhome\[0\]=([-0-9\.]+)\s+relvel=([-0-9\.]+)"
    )
    with XIAO_LOG.open() as f:
        for line in f:
            m = control_re.match(line)
            if m:
                t = int(m.group(1))
                if "enabled" in m.group(2):
                    if current:
                        current.end_ms = t
                        spans.append(current)
                    current = Span(t, None, f"span{len(spans)+1}")
                else:
                    if current:
                        current.end_ms = t
                        spans.append(current)
                        current = None
                continue
            if current:
                m = state_re.match(line)
                if m:
                    current.states.append(
                        {
                            "xiao_ms": int(m.group(1)),
                            "inav_ms": int(m.group(2)),
                            "vel_n": float(m.group(3)),
                            "vel_e": float(m.group(4)),
                            "vel_d": float(m.group(5)),
                            "qw": float(m.group(6)),
                            "qx": float(m.group(7)),
                            "qy": float(m.group(8)),
                            "qz": float(m.group(9)),
                        }
                    )
                    current.end_ms = int(m.group(2))
                    continue
                m = output_re.match(line)
                if m:
                    current.outputs.append(
                        {
                            "inav_ms": int(m.group(2)),
                            "rc0": int(m.group(3)),
                            "rc1": int(m.group(4)),
                            "rc3": int(m.group(5)),
                        }
                    )
                    continue
                m = input_re.match(line)
                if m:
                    current.inputs.append(
                        {
                            "inav_ms": int(m.group(2)),
                            "alpha": float(m.group(3)),
                            "beta": float(m.group(4)),
                            "dtheta": float(m.group(5)),
                            "dphi": float(m.group(6)),
                            "dhome": float(m.group(7)),
                            "relvel": float(m.group(8)),
                        }
                    )
    if current:
        spans.append(current)
    return spans


def build_xiao_df(span: Span) -> pd.DataFrame:
    df_s = pd.DataFrame(span.states).sort_values("inav_ms")
    df_o = pd.DataFrame(span.outputs).sort_values("inav_ms")
    df_i = pd.DataFrame(span.inputs).sort_values("inav_ms") if span.inputs else pd.DataFrame()

    # Merge outputs
    df = pd.merge_asof(df_s, df_o, on="inav_ms", direction="nearest", tolerance=50)
    df = df.dropna(subset=["rc0", "rc1", "rc3"])

    # Merge sensor inputs if available
    if not df_i.empty:
        df = pd.merge_asof(df, df_i, on="inav_ms", direction="nearest", tolerance=50)

    eulers = df.apply(lambda r: pd.Series(quat_to_euler_deg(r.qw, r.qx, r.qy, r.qz), index=["roll_deg", "pitch_deg", "yaw_deg"]), axis=1)
    df = pd.concat([df, eulers], axis=1)
    df["roll_rad"] = df["roll_deg"] * math.pi / 180.0
    df["pitch_rad"] = df["pitch_deg"] * math.pi / 180.0
    df["speed_mag"] = np.sqrt(df["vel_n"] ** 2 + df["vel_e"] ** 2 + df["vel_d"] ** 2)
    df["t_s"] = (df["inav_ms"] - df["inav_ms"].iloc[0]) * 0.001
    if len(df) >= 2:
        df["roll_rate"] = np.gradient(df["roll_rad"], df["t_s"], edge_order=2)
        df["pitch_rate"] = np.gradient(df["pitch_rad"], df["t_s"], edge_order=2)
    else:
        df["roll_rate"] = np.nan
        df["pitch_rate"] = np.nan
    return df


def _set_speed_ylim(ax_rt, series):
    if series.empty:
        return
    ymax = max(0.0, series.max())
    ax_rt.set_ylim(0.0, ymax * 1.1 if ymax > 0 else 1.0)


def _set_sym_ylim(ax, series, min_span=1.0):
    if series.empty:
        ax.set_ylim(-min_span, min_span)
        return
    m = series.abs().max()
    span = max(min_span, m * 1.1)
    ax.set_ylim(-span, span)


def _add_legend(ax, handles_left, handles_right):
    handles = handles_left + handles_right
    labels = [h.get_label() for h in handles]
    ax.legend(handles, labels, loc="upper right")


def plot_sim(df: pd.DataFrame, out: Path):
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    # common x ticks at 1s intervals
    max_t = df["time_s"].max() if not df.empty else 0
    x_ticks = np.arange(0, max_t + 0.5, 1.0)
    # Roll
    ax_l = axes[0]; ax_r = ax_l.twinx()
    h1 = ax_l.plot(df["time_s"], df["roll_cmd"], color=ROLL_COLOR, linestyle="-", label="roll cmd")
    h2 = ax_r.plot(df["time_s"], df["roll_rate"], color=ROLL_COLOR, linestyle="--", label="roll rate (rad/s)")
    ax_l.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
    ax_r.axhline(0.0, color="gray", linestyle=":", linewidth=0.8)
    ax_l.set_ylim(-1.2, 1.2); ax_r.set_ylim(-10.0, 10.0)
    ax_l.set_yticks([-1.0, -0.5, 0.0, 0.5, 1.0])
    ax_l.set_xticks(x_ticks)
    ax_l.grid(True, axis="x", linestyle=":", linewidth=0.6)
    ax_l.set_ylabel("Command"); ax_r.set_ylabel("Roll rate (rad/s)")
    _add_legend(ax_r, h1, h2)

    # Pitch
    ax_lp = axes[1]; ax_rp = ax_lp.twinx()
    h3 = ax_lp.plot(df["time_s"], df["pitch_cmd"], color=PITCH_COLOR, linestyle="-", label="pitch cmd")
    h4 = ax_rp.plot(df["time_s"], df["pitch_rate"], color=PITCH_COLOR, linestyle="--", label="pitch rate (rad/s)")
    ax_lp.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
    ax_rp.axhline(0.0, color="gray", linestyle=":", linewidth=0.8)
    ax_lp.set_ylim(-1.2, 1.2); ax_rp.set_ylim(-10.0, 10.0)
    ax_lp.set_yticks([-1.0, -0.5, 0.0, 0.5, 1.0])
    ax_lp.set_xticks(x_ticks)
    ax_lp.grid(True, axis="x", linestyle=":", linewidth=0.6)
    ax_lp.set_ylabel("Command"); ax_rp.set_ylabel("Pitch rate (rad/s)")
    _add_legend(ax_rp, h3, h4)

    # Throttle vs speed
    ax_lt = axes[2]; ax_rt = ax_lt.twinx()
    h5 = ax_lt.plot(df["time_s"], df["throttle_cmd"], color=THR_COLOR, linestyle="-", label="throttle cmd")
    h6 = ax_rt.plot(df["time_s"], df["speed_mag"], color=SPEED_COLOR, linestyle="--", label="speed (m/s)")
    ax_lt.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
    ax_lt.set_ylim(-1.2, 1.2)
    ax_lt.set_yticks([-1.0, -0.5, 0.0, 0.5, 1.0])
    ax_lt.set_xticks(x_ticks)
    ax_lt.grid(True, axis="x", linestyle=":", linewidth=0.6)
    _set_speed_ylim(ax_rt, df["speed_mag"])
    ax_lt.set_ylabel("Throttle cmd"); ax_rt.set_ylabel("Speed (m/s)")
    _add_legend(ax_rt, h5, h6)
    axes[2].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def plot_xiao(df: pd.DataFrame, out: Path, title: str):
    # Now 5 subplots: RC+rates, then GP sensors (alpha/beta, dtheta/dphi, dhome/relvel)
    fig, axes = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
    max_t = df["t_s"].max() if not df.empty else 0
    x_ticks = np.arange(0, max_t + 0.5, 1.0)
    rc_ticks = [1000, 1250, 1500, 1750, 2000]
    # Roll
    ax_l = axes[0]; ax_r = ax_l.twinx()
    h1 = ax_l.plot(df["t_s"], df["rc0"], color=ROLL_COLOR, linestyle="-", label="rc (roll)")
    h2 = ax_r.plot(df["t_s"], df["roll_rate"], color=ROLL_COLOR, linestyle="--", label="roll rate (rad/s)")
    ax_l.axhline(1500, color="gray", linestyle=":", linewidth=0.8)
    ax_r.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
    ax_l.set_ylim(950, 2050); ax_r.set_ylim(-10.0, 10.0)
    ax_l.set_yticks(rc_ticks)
    ax_l.set_xticks(x_ticks)
    ax_l.grid(True, axis="both", linestyle=":", linewidth=0.6)
    ax_l.set_ylabel("RC (µs)"); ax_r.set_ylabel("Roll rate (rad/s)")
    _add_legend(ax_r, h1, h2)

    # Pitch
    ax_lp = axes[1]; ax_rp = ax_lp.twinx()
    h3 = ax_lp.plot(df["t_s"], df["rc1"], color=PITCH_COLOR, linestyle="-", label="rc (pitch)")
    h4 = ax_rp.plot(df["t_s"], df["pitch_rate"], color=PITCH_COLOR, linestyle="--", label="pitch rate (rad/s)")
    ax_lp.axhline(1500, color="gray", linestyle=":", linewidth=0.8)
    ax_rp.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
    ax_lp.set_ylim(950, 2050); ax_rp.set_ylim(-10.0, 10.0)
    ax_lp.set_yticks(rc_ticks)
    ax_lp.set_xticks(x_ticks)
    ax_lp.grid(True, axis="both", linestyle=":", linewidth=0.6)
    ax_lp.set_ylabel("RC (µs)"); ax_rp.set_ylabel("Pitch rate (rad/s)")
    _add_legend(ax_rp, h3, h4)

    # Throttle vs speed
    ax_lt = axes[2]; ax_rt = ax_lt.twinx()
    h5 = ax_lt.plot(df["t_s"], df["rc3"], color=THR_COLOR, linestyle="-", label="rc (throttle)")
    h6 = ax_rt.plot(df["t_s"], df["speed_mag"], color=SPEED_COLOR, linestyle="--", label="speed (m/s)")
    ax_lt.axhline(1500, color="gray", linestyle=":", linewidth=0.8)
    ax_lt.set_ylim(950, 2050)
    ax_lt.set_yticks(rc_ticks)
    ax_lt.set_xticks(x_ticks)
    ax_lt.grid(True, axis="both", linestyle=":", linewidth=0.6)
    _set_speed_ylim(ax_rt, df["speed_mag"])
    ax_lt.set_ylabel("Throttle (µs)"); ax_rt.set_ylabel("Speed (m/s)")
    _add_legend(ax_rt, h5, h6)

    # GP Sensors: alpha & beta
    if "alpha" in df.columns and not df["alpha"].isna().all():
        ax3 = axes[3]
        ax3_r = ax3.twinx()
        # Plot in radians
        h7 = ax3.plot(df["t_s"], df["alpha"], color="tab:orange", linestyle="-", label="alpha (rad)")
        h8 = ax3.plot(df["t_s"], df["beta"], color="tab:purple", linestyle="-", label="beta (rad)")
        ax3.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
        ax3.set_ylabel("Alpha/Beta (rad)")
        ax3.set_xticks(x_ticks)
        ax3.grid(True, axis="both", linestyle=":", linewidth=0.6)
        # Put legend on right axis for consistency
        ax3_r.set_yticks([])  # Hide right axis ticks
        _add_legend(ax3_r, h7 + h8, [])
    else:
        axes[3].text(0.5, 0.5, "No GP Input data", ha="center", va="center", transform=axes[3].transAxes)
        axes[3].set_ylabel("Alpha/Beta")

    # GP Sensors: dtheta, dphi, dhome
    if "dtheta" in df.columns and not df["dtheta"].isna().all():
        ax4 = axes[4]
        ax4_r = ax4.twinx()
        # Plot angles in radians on left axis
        h9 = ax4.plot(df["t_s"], df["dtheta"], color="tab:cyan", linestyle="-", label="dtheta (rad)")
        h10 = ax4.plot(df["t_s"], df["dphi"], color="tab:pink", linestyle="-", label="dphi (rad)")
        # dhome in meters on right axis
        h11 = ax4_r.plot(df["t_s"], df["dhome"], color="tab:brown", linestyle="--", label="dhome (m)")
        ax4.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)
        ax4_r.axhline(0.0, color="gray", linestyle=":", linewidth=0.8)
        ax4.set_ylabel("Dtheta/Dphi (rad)")
        ax4_r.set_ylabel("Dhome (m)")
        ax4.set_xticks(x_ticks)
        ax4.grid(True, axis="both", linestyle=":", linewidth=0.6)
        _add_legend(ax4_r, h9 + h10, h11)
    else:
        axes[4].text(0.5, 0.5, "No GP Input data", ha="center", va="center", transform=axes[4].transAxes)
        axes[4].set_ylabel("Dtheta/Dphi/Dhome")

    axes[4].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def main():
    sim_df = load_sim()
    if not sim_df.empty:
        plot_sim(sim_df, ROOT / "control_response_sim.png")

    spans = parse_xiao_spans()
    for span in spans:
        df = build_xiao_df(span)
        if df.empty:
            continue
        plot_xiao(df, ROOT / f"control_response_xiao_{span.label}.png", title=f"Control Response – Xiao {span.label}")


if __name__ == "__main__":
    main()
