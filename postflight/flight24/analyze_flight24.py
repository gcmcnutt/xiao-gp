#!/usr/bin/env python3
"""
Flight24 postflight helper.
- Parse GP Control spans from the xiao log
- Correlate GP Output RC commands to INAV rcData in the blackbox CSV
- Apply the new origin offset convention (origin + [0,0,-25]) to INAV positions
- Generate per-span overlays for position/velocity/quaternion and RC
Outputs per-span timing and rcData deltas to spot latency/interpretation issues.

IMPORTANT COORDINATE NOTE (2025-12-20):
- INAV uses NED coordinates (NOT NEU as previously thought)
- INAV blackbox logs contain body->earth quaternions
- msplink.cpp applies conjugate to convert to earth->body for GP
- This script processes raw INAV data (body->earth quaternions)
"""

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
BLACKBOX_CSV = Path("/home/gmcnutt/GP/autoc/flight24.csv")
XIAO_LOG = Path("/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/fligt24-hb1-flight_log_2025-12-21T18-10-34.txt")
# How far forward to search (microseconds) for the commanded RC to appear in INAV
FORWARD_WINDOW_US = 50_000  # 50 ms
# Origin offset applied at arming (virtual origin at NED = [0, 0, -25])
ORIGIN_OFFSET_NED = (0.0, 0.0, -25.0)


@dataclass
class Span:
    start_ms: int
    end_ms: int
    label: str
    origin_ned: Optional[tuple] = None
    evals: List[dict] = field(default_factory=list)
    states: List[dict] = field(default_factory=list)
    debugs: List[dict] = field(default_factory=list)
    inputs: List[dict] = field(default_factory=list)


def load_blackbox() -> pd.DataFrame:
    df = pd.read_csv(BLACKBOX_CSV, skipinitialspace=True)
    rename = {
        "time (us)": "time_us",
        "rcData[0]": "rc0",
        "rcData[1]": "rc1",
        "rcData[3]": "rc3",
        "navPos[0]": "pos_n",
        "navPos[1]": "pos_e",
        "navPos[2]": "pos_u",
        "navVel[0]": "vel_n",
        "navVel[1]": "vel_e",
        "navVel[2]": "vel_u",
        "quaternion[0]": "qw_raw",
        "quaternion[1]": "qx_raw",
        "quaternion[2]": "qy_raw",
        "quaternion[3]": "qz_raw",
    }
    df = df.rename(columns=rename)
    df = df[
        [
            "time_us",
            "rc0",
            "rc1",
            "rc3",
            "pos_n",
            "pos_e",
            "pos_u",
            "vel_n",
            "vel_e",
            "vel_u",
            "qw_raw",
            "qx_raw",
            "qy_raw",
            "qz_raw",
        ]
    ].copy()
    # Unit conversions: cm->m, navPos[2] is UP so negate to get DOWN, quats scaled /10000
    # NOTE: INAV is NED but navPos[2] = altitude (up), so we negate to get down
    df["pos_n"] = df["pos_n"] / 100.0
    df["pos_e"] = df["pos_e"] / 100.0
    df["pos_d"] = -df["pos_u"] / 100.0  # navPos[2] is altitude (up), negate for NED down
    df["vel_n"] = df["vel_n"] / 100.0
    df["vel_e"] = df["vel_e"] / 100.0
    df["vel_d"] = -df["vel_u"] / 100.0  # navVel[2] is climb rate (up), negate for NED down
    df["qw"] = df["qw_raw"] / 10000.0
    df["qx"] = df["qx_raw"] / 10000.0
    df["qy"] = df["qy_raw"] / 10000.0
    df["qz"] = df["qz_raw"] / 10000.0
    df["vel_mag"] = np.sqrt(df["vel_n"] ** 2 + df["vel_e"] ** 2 + df["vel_d"] ** 2)
    return df


def parse_xiao_log() -> List[Span]:
    spans: List[Span] = []
    current: Optional[Span] = None
    control_re = re.compile(
        r"^#[0-9]+\s+([0-9]+)\s+([0-9]+)\s+[iwe]\s+GP Control: (Switch enabled|Switch disabled).*"
    )
    origin_re = re.compile(r"(path|test) origin NED=\[([-0-9\\.]+),\s*([-0-9\\.]+),\s*([-0-9\\.]+)\]")
    output_re = re.compile(r"^#[0-9]+\s+([0-9]+)\s+([0-9]+)\s+[iwe]\s+GP (?:Cycle )?Output:.*rc=\[(\d+),(\d+),(\d+)\]")
    input_re = re.compile(
        r"^#[0-9]+\s+([0-9]+)\s+([0-9]+)\s+[iwe]\s+GP Input: idx=(\d+) rabbit=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\] vec=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\]\s+alpha\[0\]=([-0-9\.]+)\s+beta\[0\]=([-0-9\.]+)\s+dtheta\[0\]=([-0-9\.]+)\s+dphi\[0\]=([-0-9\.]+)\s+dhome\[0\]=([-0-9\.]+)\s+relvel=([-0-9\.]+)"
    )
    state_re = re.compile(
        r"^#[0-9]+\s+([0-9]+)\s+([0-9]+)\s+[iwe]\s+GP State: pos_raw=\[([-0-9\\.]+),([-0-9\\.]+),([-0-9\\.]+)\]\s+pos=\[([-0-9\\.]+),([-0-9\\.]+),([-0-9\\.]+)\]\s+vel=\[([-0-9\\.]+),([-0-9\\.]+),([-0-9\\.]+)\].*quat=\[([-0-9\\.]+),([-0-9\\.]+),([-0-9\\.]+),([-0-9\\.]+)\]"
    )
    autoc_flag_re = re.compile(r"autoc=(Y|N)")

    with XIAO_LOG.open() as f:
        for line in f:
            m = control_re.match(line)
            if m:
                inav_ms = int(m.group(2))
                state = m.group(3)
                if state == "Switch enabled":
                    if current and current.end_ms is None:
                        current.end_ms = inav_ms
                        spans.append(current)
                    idx = len(spans) + 1
                    origin = None
                    o = origin_re.search(line)
                    if o:
                        origin = (float(o.group(2)), float(o.group(3)), float(o.group(4)))
                    current = Span(start_ms=inav_ms, end_ms=None, label=f"test{idx}", origin_ned=origin)
                else:
                    if current:
                        current.end_ms = inav_ms
                        spans.append(current)
                        current = None
                continue

            m = output_re.match(line)
            if m:
                inav_ms = int(m.group(2))
                rc = tuple(int(m.group(i)) for i in range(3, 6))
                entry = {"inav_ms": inav_ms, "rc": rc}
                if current is not None:
                    current.evals.append(entry)
                continue

            m = input_re.match(line)
            if m and current is not None:
                inav_ms = int(m.group(2))
                entry = {
                    "inav_ms": inav_ms,
                    "idx": int(m.group(3)),
                    "rabbit": (float(m.group(4)), float(m.group(5)), float(m.group(6))),
                    "vec": (float(m.group(7)), float(m.group(8)), float(m.group(9))),
                    "alpha": float(m.group(10)),
                    "beta": float(m.group(11)),
                    "dtheta": float(m.group(12)),
                    "dphi": float(m.group(13)),
                    "dhome": float(m.group(14)),
                    "relvel": float(m.group(15)),
                }
                current.inputs.append(entry)
                continue

            m = state_re.match(line)
            if m and current is not None:
                autoc_match = autoc_flag_re.search(line)
                autoc_flag = autoc_match.group(1) if autoc_match else None
                current.states.append(
                    {
                        "xiao_ms": int(m.group(1)),
                        "inav_ms": int(m.group(2)),
                        "pos_raw_n": float(m.group(3)),
                        "pos_raw_e": float(m.group(4)),
                        "pos_raw_d": float(m.group(5)),
                        "pos_n": float(m.group(6)),  # GP-relative pos (includes offset)
                        "pos_e": float(m.group(7)),
                        "pos_d": float(m.group(8)),
                        "vel_n": float(m.group(9)),
                        "vel_e": float(m.group(10)),
                        "vel_d": float(m.group(11)),
                        "qw": float(m.group(12)),
                        "qx": float(m.group(13)),
                        "qy": float(m.group(14)),
                        "qz": float(m.group(15)),
                        "vel_mag": None,  # fill later
                        "autoc": autoc_flag,
                    }
                )
                if autoc_flag == "N" and current and current.end_ms is None:
                    current.end_ms = int(m.group(2))
                    spans.append(current)
                    current = None
                continue

    if current:
        last_time = current.evals[-1]["inav_ms"] if current.evals else (current.states[-1]["inav_ms"] if current.states else current.start_ms)
        current.end_ms = last_time
        spans.append(current)
    return spans


def find_nearest_indices(bb_times: np.ndarray, target_us: np.ndarray) -> np.ndarray:
    idxs = np.searchsorted(bb_times, target_us, side="left")
    idxs = np.clip(idxs, 0, len(bb_times) - 1)
    mask = (idxs > 0) & (idxs < len(bb_times))
    left = np.abs(bb_times[idxs[mask] - 1] - target_us[mask])
    right = np.abs(bb_times[idxs[mask]] - target_us[mask])
    idxs[mask] -= (left < right)
    return idxs


def summarize_span(span: Span, bb: pd.DataFrame):
    if not span.evals:
        return None
    eval_times_us = np.array([e["inav_ms"] * 1000 for e in span.evals])
    bb_times = bb["time_us"].to_numpy()
    nearest = find_nearest_indices(bb_times, eval_times_us)
    bb_rows = bb.iloc[nearest]
    rc_set = np.array([e["rc"] for e in span.evals])
    rc_bb = bb_rows[["rc0", "rc1", "rc3"]].to_numpy()
    deltas = rc_bb - rc_set
    lag_us = (bb_rows["time_us"].to_numpy() - eval_times_us).astype(int)
    summary = {
        "span": span.label,
        "start_ms": span.start_ms,
        "end_ms": span.end_ms,
        "samples": len(span.evals),
        "lag_us_min": int(lag_us.min()),
        "lag_us_mean": float(lag_us.mean()),
        "lag_us_max": int(lag_us.max()),
        "rc_abs_mean": np.abs(deltas.astype(float)).mean(axis=0).round(1).tolist(),
        "rc_abs_max": np.abs(deltas.astype(float)).max(axis=0).tolist(),
    }

    window = FORWARD_WINDOW_US
    forward_lags = []
    forward_deltas = []
    for t_us, cmd in zip(eval_times_us, rc_set):
        mask = (bb_times >= t_us) & (bb_times <= t_us + window)
        if not mask.any():
            continue
        window_rows = bb.loc[mask, ["time_us", "rc0", "rc1", "rc3"]]
        rc_vals = window_rows[["rc0", "rc1", "rc3"]].to_numpy()
        diff = np.abs(rc_vals - cmd)
        score = diff.sum(axis=1)
        best_idx = score.argmin()
        forward_lags.append(int(window_rows.iloc[best_idx]["time_us"] - t_us))
        forward_deltas.append(diff[best_idx])

    if forward_lags:
        fd = np.vstack(forward_deltas)
        summary.update(
            forward_samples=len(forward_lags),
            forward_lag_us_min=int(np.min(forward_lags)),
            forward_lag_us_mean=float(np.mean(forward_lags)),
            forward_lag_us_max=int(np.max(forward_lags)),
            forward_rc_abs_mean=np.round(fd.mean(axis=0), 1).tolist(),
            forward_rc_abs_max=fd.max(axis=0).tolist(),
        )
    return summary


def apply_origin_to_bb(bb: pd.DataFrame, origin: tuple) -> pd.DataFrame:
    """Apply origin shift and the fixed offset to BB positions."""
    n_off, e_off, d_off = ORIGIN_OFFSET_NED
    bb = bb.copy()
    bb["pos_n_rel"] = bb["pos_n"] - origin[0] + n_off
    bb["pos_e_rel"] = bb["pos_e"] - origin[1] + e_off
    bb["pos_d_rel"] = bb["pos_d"] - origin[2] + d_off
    bb["vel_mag"] = np.sqrt(bb["vel_n"] ** 2 + bb["vel_e"] ** 2 + bb["vel_d"] ** 2)
    return bb


def generate_overview_plots(spans: List[Span], bb: pd.DataFrame):
    plot_idx = 1
    for span in spans:
        if not span.states or not span.origin_ned:
            continue
        df = pd.DataFrame(span.states)
        rc_df = pd.DataFrame(span.evals)
        inputs_df = pd.DataFrame(span.inputs)
        origin = span.origin_ned
        t0 = span.start_ms
        df["t_rel"] = (df["inav_ms"] - t0) / 1000.0
        df["vel_mag"] = np.sqrt(df["vel_n"] ** 2 + df["vel_e"] ** 2 + df["vel_d"] ** 2)
        if not rc_df.empty:
            rc_df["t_rel"] = (rc_df["inav_ms"] - t0) / 1000.0
            rc_df[["rc0_cmd", "rc1_cmd", "rc3_cmd"]] = pd.DataFrame(rc_df["rc"].to_list(), index=rc_df.index)
        if not inputs_df.empty:
            inputs_df["t_rel"] = (inputs_df["inav_ms"] - t0) / 1000.0
            inputs_df["alpha_deg"] = np.degrees(inputs_df["alpha"])
            inputs_df["beta_deg"] = np.degrees(inputs_df["beta"])
            inputs_df["dtheta_deg"] = np.degrees(inputs_df["dtheta"])
            inputs_df["dphi_deg"] = np.degrees(inputs_df["dphi"])
        # Recomputed sensors for overlay (dots)
        recomp = []
        if not inputs_df.empty:
            df_states = pd.DataFrame(span.states).sort_values("inav_ms").reset_index(drop=True)
            for _, row in inputs_df.iterrows():
                inav_ms = row["inav_ms"]
                idx = (df_states["inav_ms"] - inav_ms).abs().idxmin()
                st = df_states.loc[idx]
                qw, qx, qy, qz = st["qw"], st["qx"], st["qy"], st["qz"]
                R = quat_to_rotmat(qw, qx, qy, qz)
                v_world = np.array([st["vel_n"], st["vel_e"], st["vel_d"]])
                v_body = R.T @ v_world
                alpha = math.atan2(-v_body[2], v_body[0])
                beta = math.atan2(v_body[1], v_body[0])
                rabbit = np.array(row["rabbit"])
                pos = np.array([st["pos_n"], st["pos_e"], st["pos_d"]])
                craft_to_target = rabbit - pos
                target_body = R.T @ craft_to_target
                dtheta = math.atan2(-target_body[2], target_body[0])
                dphi = math.atan2(target_body[1], -target_body[2])  # match executeGetDPhi (YZ plane vs body Z)
                recomp.append(
                    {
                        "t_rel": (inav_ms - t0) / 1000.0,
                        "alpha_deg": math.degrees(alpha),
                        "beta_deg": math.degrees(beta),
                        "dtheta_deg": math.degrees(dtheta),
                        "dphi_deg": math.degrees(dphi),
                        "dhome": np.linalg.norm(np.array([0.0, 0.0, -25.0]) - pos),
                    }
                )
        recomp_df = pd.DataFrame(recomp)

        win = (bb["time_us"] / 1000.0 >= t0) & (bb["time_us"] / 1000.0 <= (span.end_ms or t0))
        bb_win = bb.loc[win].copy()
        bb_win["t_rel"] = (bb_win["time_us"] / 1000.0 - t0) / 1000.0
        bb_win = apply_origin_to_bb(bb_win, origin)

        fig, axes = plt.subplots(5, 1, figsize=(10, 14), sharex=True)
        colors = ["#d62728", "#2ca02c", "#1f77b4", "#9467bd"]

        axes[0].grid(True, linestyle="--", linewidth=0.4)
        axes[0].plot(bb_win["t_rel"], bb_win["pos_n_rel"], color=colors[0], linewidth=1.0, label="INAV N")
        axes[0].plot(bb_win["t_rel"], bb_win["pos_e_rel"], color=colors[1], linewidth=1.0, label="INAV E")
        axes[0].plot(bb_win["t_rel"], bb_win["pos_d_rel"], color=colors[2], linewidth=1.0, label="INAV D")
        axes[0].scatter(df["t_rel"], df["pos_n"], s=12, color=colors[0], marker="o", label="xiao N (GP State)")
        axes[0].scatter(df["t_rel"], df["pos_e"], s=12, color=colors[1], marker="o", label="xiao E (GP State)")
        axes[0].scatter(df["t_rel"], df["pos_d"], s=12, color=colors[2], marker="o", label="xiao D (GP State)")
        axes[0].axvline(0, color="k", linestyle="--", linewidth=0.8)
        axes[0].set_ylabel("Pos (m)")
        axes[0].legend(loc="upper right", fontsize="xx-small", ncol=2)

        axes[1].grid(True, linestyle="--", linewidth=0.4)
        axes[1].plot(bb_win["t_rel"], bb_win["vel_n"], color=colors[0], linewidth=1.0, label="INAV Vn")
        axes[1].plot(bb_win["t_rel"], bb_win["vel_e"], color=colors[1], linewidth=1.0, label="INAV Ve")
        axes[1].plot(bb_win["t_rel"], bb_win["vel_d"], color=colors[2], linewidth=1.0, label="INAV Vd")
        axes[1].scatter(df["t_rel"], df["vel_n"], s=12, color=colors[0], marker="o", label="xiao Vn")
        axes[1].scatter(df["t_rel"], df["vel_e"], s=12, color=colors[1], marker="o", label="xiao Ve")
        axes[1].scatter(df["t_rel"], df["vel_d"], s=12, color=colors[2], marker="o", label="xiao Vd")
        axes[1].plot(bb_win["t_rel"], bb_win["vel_mag"], color="black", linewidth=1.0, linestyle="--", label="INAV |V|")
        axes[1].scatter(df["t_rel"], df["vel_mag"], s=10, color="black", marker="x", label="xiao |V|")
        axes[1].axvline(0, color="k", linestyle="--", linewidth=0.8)
        axes[1].set_ylabel("Vel (m/s)")
        axes[1].legend(loc="upper right", fontsize="xx-small", ncol=2)

        axes[2].grid(True, linestyle="--", linewidth=0.4)
        axes[2].plot(bb_win["t_rel"], bb_win["qw"], color=colors[0], linewidth=1.0, label="INAV qw")
        axes[2].plot(bb_win["t_rel"], bb_win["qx"], color=colors[1], linewidth=1.0, label="INAV qx")
        axes[2].plot(bb_win["t_rel"], bb_win["qy"], color=colors[2], linewidth=1.0, label="INAV qy")
        axes[2].plot(bb_win["t_rel"], bb_win["qz"], color=colors[3], linewidth=1.0, label="INAV qz")
        axes[2].scatter(df["t_rel"], df["qw"], s=12, color=colors[0], marker="o", label="xiao qw")
        axes[2].scatter(df["t_rel"], df["qx"], s=12, color=colors[1], marker="o", label="xiao qx")
        axes[2].scatter(df["t_rel"], df["qy"], s=12, color=colors[2], marker="o", label="xiao qy")
        axes[2].scatter(df["t_rel"], df["qz"], s=12, color=colors[3], marker="o", label="xiao qz")
        axes[2].axvline(0, color="k", linestyle="--", linewidth=0.8)
        axes[2].set_ylabel("Quat")
        axes[2].legend(loc="upper right", fontsize="xx-small", ncol=2)

        axes[3].grid(True, linestyle="--", linewidth=0.4)
        axes[3].plot(bb_win["t_rel"], bb_win["rc0"], color=colors[0], linewidth=1.0, label="INAV rc0")
        axes[3].plot(bb_win["t_rel"], bb_win["rc1"], color=colors[1], linewidth=1.0, label="INAV rc1")
        axes[3].plot(bb_win["t_rel"], bb_win["rc3"], color=colors[2], linewidth=1.0, label="INAV rc3")
        if not rc_df.empty:
            axes[3].scatter(rc_df["t_rel"], rc_df["rc0_cmd"], s=14, color=colors[0], marker="o", label="cmd rc0")
            axes[3].scatter(rc_df["t_rel"], rc_df["rc1_cmd"], s=14, color=colors[1], marker="o", label="cmd rc1")
            axes[3].scatter(rc_df["t_rel"], rc_df["rc3_cmd"], s=14, color=colors[2], marker="o", label="cmd rc3")
        axes[3].axvline(0, color="k", linestyle="--", linewidth=0.8)
        for y in (1000, 1500, 2000):
            axes[3].axhline(y, color="gray", linestyle=":", linewidth=0.6)
        axes[3].set_ylabel("RC us")
        axes[3].set_xlabel("Time since span start (s)")
        axes[3].legend(loc="upper right", fontsize="xx-small", ncol=2)

        # Sensors (alpha/beta/dtheta/dphi, deg) + dhome (m)
        axes[4].grid(True, linestyle="--", linewidth=0.4)
        if not recomp_df.empty:
            axes[4].plot(recomp_df["t_rel"], recomp_df["alpha_deg"], color=colors[0], linewidth=1.0, label="alpha recompute (deg)")
            axes[4].plot(recomp_df["t_rel"], recomp_df["beta_deg"], color=colors[1], linewidth=1.0, label="beta recompute (deg)")
            axes[4].plot(recomp_df["t_rel"], recomp_df["dtheta_deg"], color=colors[2], linewidth=1.0, label="dtheta recompute (deg)")
            axes[4].plot(recomp_df["t_rel"], recomp_df["dphi_deg"], color=colors[3], linewidth=1.0, label="dphi recompute (deg)")
        if not inputs_df.empty:
            axes[4].scatter(inputs_df["t_rel"], inputs_df["alpha_deg"], s=24, color=colors[0], alpha=0.7, edgecolors="black", linewidths=0.4, marker="o", label="alpha log (deg)", zorder=5)
            axes[4].scatter(inputs_df["t_rel"], inputs_df["beta_deg"], s=24, color=colors[1], alpha=0.7, edgecolors="black", linewidths=0.4, marker="o", label="beta log (deg)", zorder=5)
            axes[4].scatter(inputs_df["t_rel"], inputs_df["dtheta_deg"], s=24, color=colors[2], alpha=0.7, edgecolors="black", linewidths=0.4, marker="o", label="dtheta log (deg)", zorder=5)
            axes[4].scatter(inputs_df["t_rel"], inputs_df["dphi_deg"], s=24, color=colors[3], alpha=0.7, edgecolors="black", linewidths=0.4, marker="o", label="dphi log (deg)", zorder=5)
        axes[4].axvline(0, color="k", linestyle="--", linewidth=0.8)
        axes[4].set_ylabel("Angles (deg)")
        # dhome on twin axis
        if not inputs_df.empty or not recomp_df.empty:
            ax_dhome = axes[4].twinx()
            if not recomp_df.empty:
                ax_dhome.plot(recomp_df["t_rel"], recomp_df["dhome"], color="black", linestyle="--", linewidth=1.0, label="dhome recompute (m)")
            if not inputs_df.empty:
                ax_dhome.scatter(inputs_df["t_rel"], inputs_df["dhome"], s=24, color="gray", alpha=0.7, marker="x", label="dhome log (m)", zorder=5)
            ax_dhome.set_ylabel("dhome (m)")
            lines, labels = axes[4].get_legend_handles_labels()
            lines2, labels2 = ax_dhome.get_legend_handles_labels()
            axes[4].legend(lines + lines2, labels + labels2, loc="upper right", fontsize="xx-small", ncol=2)
        else:
            axes[4].legend(loc="upper right", fontsize="xx-small", ncol=2)

        plt.tight_layout()
        plt.savefig(ROOT / f"span{plot_idx}_overview.png", dpi=150)
        plt.close()
        plot_idx += 1

    print("Per-span overview plots written to", ROOT)


def quat_to_rotmat(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Return body->world rotation matrix from quaternion (w,x,y,z)."""
    ww, xx, yy, zz = qw, qx, qy, qz
    return np.array(
        [
            [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww), 2 * (xx * zz + yy * ww)],
            [2 * (xx * yy + zz * ww), 1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
            [2 * (xx * zz - yy * ww), 2 * (yy * zz + xx * ww), 1 - 2 * (xx * xx + yy * yy)],
        ]
    )


def compare_gp_inputs(spans: List[Span]):
    """Recompute alpha/beta/dtheta/dphi/dhome from state to confirm parity with autoc."""
    print("\nGP input parity check (recomputed vs logged):")
    for span in spans:
        if not span.inputs or not span.states:
            continue
        df_states = pd.DataFrame(span.states).sort_values("inav_ms").reset_index(drop=True)
        mismatches = []
        for entry in span.inputs:
            inav_ms = entry["inav_ms"]
            idx = (df_states["inav_ms"] - inav_ms).abs().idxmin()
            st = df_states.loc[idx]
            qw, qx, qy, qz = st["qw"], st["qx"], st["qy"], st["qz"]
            R = quat_to_rotmat(qw, qx, qy, qz)
            v_world = np.array([st["vel_n"], st["vel_e"], st["vel_d"]])
            v_body = R.T @ v_world  # world->body if quat is body->world
            alpha = math.atan2(-v_body[2], v_body[0])  # rad
            beta = math.atan2(v_body[1], v_body[0])
            rabbit = np.array(entry["rabbit"])
            pos = np.array([st["pos_n"], st["pos_e"], st["pos_d"]])
            craft_to_target = rabbit - pos
            target_body = R.T @ craft_to_target
            dtheta = math.atan2(-target_body[2], target_body[0])
            # Match executeGetDPhi: project onto body YZ plane, angle vs body Z
            dphi = math.atan2(target_body[1], -target_body[2])
            home = np.array([0.0, 0.0, -25.0])
            dhome = np.linalg.norm(home - pos)
            def wrap(a: float) -> float:
                return (a + math.pi) % (2 * math.pi) - math.pi
            diffs = {
                "alpha": wrap(alpha - entry["alpha"]),
                "beta": wrap(beta - entry["beta"]),
                "dtheta": wrap(dtheta - entry["dtheta"]),
                "dphi": wrap(dphi - entry["dphi"]),
                "dhome": dhome - entry["dhome"],
            }
            mismatches.append(diffs)
        if mismatches:
            df = pd.DataFrame(mismatches)
            df_deg = df[["alpha", "beta", "dtheta", "dphi"]].applymap(lambda x: x * 180.0 / math.pi)
            print(
                f"- {span.label}: abs mean diff deg (alpha/beta/dtheta/dphi) = "
                f"{df_deg.abs().mean().round(3).tolist()}, "
                f"dhome mean diff={df['dhome'].mean():.3f} m, max diff deg={df_deg.abs().max().round(3).to_dict()}"
            )


def main():
    bb = load_blackbox()
    spans = parse_xiao_log()
    print("Detected spans:")
    for s in spans:
        duration = (s.end_ms or s.start_ms) - s.start_ms
        print(f"  {s.label}: {s.start_ms}–{s.end_ms} ms ({duration/1000:.2f}s), evals={len(s.evals)} origin={s.origin_ned}")

    print("\nRC correlation (xiao setRcData/Output -> INAV rcData nearest sample):")
    summaries = [summarize_span(s, bb) for s in spans]
    summaries = [s for s in summaries if s]
    for s in summaries:
        roll_mean, pitch_mean, thr_mean = s["rc_abs_mean"]
        roll_max, pitch_max, thr_max = s["rc_abs_max"]
        line = (
            f"- {s['span']}: samples={s['samples']} lag_us(min/mean/max) "
            f"{s['lag_us_min']}/{s['lag_us_mean']:.1f}/{s['lag_us_max']} | "
            f"rc |mean|=[{roll_mean},{pitch_mean},{thr_mean}] "
            f"max=[{roll_max},{pitch_max},{thr_max}]"
        )
        if "forward_samples" in s:
            fr_mean = s["forward_rc_abs_mean"]
            fr_max = s["forward_rc_abs_max"]
            line += (
                f" | forward window {FORWARD_WINDOW_US/1000:.0f}ms: samples={s['forward_samples']} "
                f"lag_us(min/mean/max) {s['forward_lag_us_min']}/"
                f"{s['forward_lag_us_mean']:.1f}/{s['forward_lag_us_max']} "
                f"| rc |mean|=[{fr_mean[0]},{fr_mean[1]},{fr_mean[2]}] "
                f"max=[{fr_max[0]},{fr_max[1]},{fr_max[2]}]"
            )
        print(line)

    generate_overview_plots(spans, bb)
    compare_gp_inputs(spans)


if __name__ == "__main__":
    main()
