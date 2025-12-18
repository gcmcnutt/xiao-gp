#!/usr/bin/env python3
"""
Bench14 postflight helper.
- Parse GP Control spans from the xiao log
- Correlate GP Output RC commands to INAV rcData in the blackbox CSV
- Generate per-span overlays for position/velocity/quaternion and RC
Outputs per-span timing and rcData deltas to spot latency/interpretation issues.
"""

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
BLACKBOX_CSV = Path("/home/gmcnutt/GP/autoc/bench14.csv")
XIAO_LOG = Path("/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/bench14-hb1-flight_log_2025-12-18T04-49-08.txt")
# How far forward to search (microseconds) for the commanded RC to appear in INAV
FORWARD_WINDOW_US = 50_000  # 50 ms


@dataclass
class Span:
    start_ms: int
    end_ms: int
    label: str
    origin_ned: Optional[tuple] = None
    evals: List[dict] = field(default_factory=list)
    states: List[dict] = field(default_factory=list)
    debugs: List[dict] = field(default_factory=list)


def load_blackbox() -> pd.DataFrame:
    df = pd.read_csv(BLACKBOX_CSV, skipinitialspace=True)
    # Normalize field names we care about. For this airframe: rc0=roll, rc1=pitch,
    # rc3=throttle (rc2 is rudder, unused here).
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
    # Unit conversions: cm->m, NEU->NED (flip up to down), quats scaled /10000
    df["pos_n"] = df["pos_n"] / 100.0
    df["pos_e"] = df["pos_e"] / 100.0
    df["pos_d"] = -df["pos_u"] / 100.0
    df["vel_n"] = df["vel_n"] / 100.0
    df["vel_e"] = df["vel_e"] / 100.0
    df["vel_d"] = -df["vel_u"] / 100.0
    df["qw"] = df["qw_raw"] / 10000.0
    df["qx"] = df["qx_raw"] / 10000.0
    df["qy"] = df["qy_raw"] / 10000.0
    df["qz"] = df["qz_raw"] / 10000.0
    df["vel_mag"] = np.sqrt(df["vel_n"] ** 2 + df["vel_e"] ** 2 + df["vel_d"] ** 2)
    return df


def align_blackbox_time(bb: pd.DataFrame, spans: List[Span]) -> pd.DataFrame:
    """Shift blackbox time_us so that the first sample aligns with the first span start."""
    if not spans or bb.empty:
        bb["time_us_aligned"] = bb["time_us"]
        return bb
    offset_us = bb["time_us"].iloc[0] - spans[0].start_ms * 1000
    bb["time_us_aligned"] = bb["time_us"] - offset_us
    return bb


def parse_xiao_log() -> List[Span]:
    spans: List[Span] = []
    current: Optional[Span] = None
    control_re = re.compile(
        r"^#[0-9]+\s+([0-9]+)\s+([0-9]+)\s+[iwe]\s+GP Control: (Switch enabled|Switch disabled).*"
    )
    origin_re = re.compile(r"(path|test) origin NED=\[([-0-9\\.]+),\s*([-0-9\\.]+),\s*([-0-9\\.]+)\]")
    output_re = re.compile(r"^#[0-9]+\s+([0-9]+)\s+([0-9]+)\s+[iwe]\s+GP (?:Cycle )?Output:.*rc=\[(\d+),(\d+),(\d+)\]")
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
                    # Close any dangling span
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
                        "pos_n": float(m.group(6)),  # relative pos seen by GP
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

    # If last span never got an explicit disable, close it at last eval time
    if current:
        last_time = current.evals[-1]["inav_ms"] if current.evals else (current.states[-1]["inav_ms"] if current.states else current.start_ms)
        current.end_ms = last_time
        spans.append(current)
    return spans


def find_nearest_indices(bb_times: np.ndarray, target_us: np.ndarray) -> np.ndarray:
    # bb_times must be sorted ascending
    idxs = np.searchsorted(bb_times, target_us, side="left")
    idxs = np.clip(idxs, 0, len(bb_times) - 1)
    # For points not at edges, choose closer neighbor
    mask = (idxs > 0) & (idxs < len(bb_times))
    left = np.abs(bb_times[idxs[mask] - 1] - target_us[mask])
    right = np.abs(bb_times[idxs[mask]] - target_us[mask])
    idxs[mask] -= (left < right)
    return idxs


def summarize_span(span: Span, bb: pd.DataFrame):
    if not span.evals:
        return None
    eval_times_us = np.array([e["inav_ms"] * 1000 for e in span.evals])
    bb_times = bb["time_us_aligned"].to_numpy()
    nearest = find_nearest_indices(bb_times, eval_times_us)
    bb_rows = bb.iloc[nearest]
    rc_set = np.array([e["rc"] for e in span.evals])
    rc_bb = bb_rows[["rc0", "rc1", "rc3"]].to_numpy()
    deltas = rc_bb - rc_set
    lag_us = (bb_rows["time_us_aligned"].to_numpy() - eval_times_us).astype(int)
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

    # Forward search: find first INAV sample within window that best matches commanded RC
    window = FORWARD_WINDOW_US
    forward_lags = []
    forward_deltas = []
    for t_us, cmd in zip(eval_times_us, rc_set):
        mask = (bb_times >= t_us) & (bb_times <= t_us + window)
        if not mask.any():
            continue
        window_rows = bb.loc[mask, ["time_us_aligned", "rc0", "rc1", "rc3"]]
        rc_vals = window_rows[["rc0", "rc1", "rc3"]].to_numpy()
        diff = np.abs(rc_vals - cmd)
        # pick row with smallest L1 diff
        score = diff.sum(axis=1)
        best_idx = score.argmin()
        forward_lags.append(int(window_rows.iloc[best_idx]["time_us_aligned"] - t_us))
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


def velocity_summary(span: Span, bb: pd.DataFrame):
    if not span.states:
        return None
    sdf = pd.DataFrame(span.states)
    times_us = sdf["inav_ms"].to_numpy() * 1000
    bb_times = bb["time_us_aligned"].to_numpy()
    idx = find_nearest_indices(bb_times, times_us)
    bb_rows = bb.iloc[idx]
    deltas = {}
    for comp in ["vel_n", "vel_e", "vel_d"]:
        diff = bb_rows[comp].to_numpy() - sdf[comp].to_numpy()
        deltas[comp] = {
            "mean": float(diff.mean()),
            "mean_abs": float(np.abs(diff).mean()),
            "min": float(diff.min()),
            "max": float(diff.max()),
        }
    return deltas


def generate_overview_plots(spans: List[Span], bb: pd.DataFrame):
    for idx, span in enumerate(spans, 1):
        if not span.states:
            continue
        df = pd.DataFrame(span.states)
        rc_df = pd.DataFrame(span.evals)
        dbg_df = pd.DataFrame(span.states)  # use GP State pos for craft dots
        # Positions: xiao logs are already origin-adjusted (pos_*). Keep raw for reference if needed.
        dbg_df["pos_n_rel"] = df["pos_n"]
        dbg_df["pos_e_rel"] = df["pos_e"]
        dbg_df["pos_d_rel"] = df["pos_d"]
        origin = span.origin_ned or (0.0, 0.0, 0.0)
        t0 = span.start_ms
        df["t_rel"] = (df["inav_ms"] - t0) / 1000.0
        if not rc_df.empty:
            rc_df["t_rel"] = (rc_df["inav_ms"] - t0) / 1000.0
            rc_df[["rc0_cmd", "rc1_cmd", "rc3_cmd"]] = pd.DataFrame(rc_df["rc"].to_list(), index=rc_df.index)
        if not dbg_df.empty:
            dbg_df["t_rel"] = (dbg_df["inav_ms"] - t0) / 1000.0
        # xiao positions are logged post-origin-reset; INAV needs offset applied
        df["vel_mag"] = np.sqrt(df["vel_n"] ** 2 + df["vel_e"] ** 2 + df["vel_d"] ** 2)

        # BB window
        win = (bb["time_us_aligned"] / 1000.0 >= t0) & (bb["time_us_aligned"] / 1000.0 <= (span.end_ms or t0))
        bb_win = bb.loc[win].copy()
        bb_win["t_rel"] = (bb_win["time_us_aligned"] / 1000.0 - t0) / 1000.0
        # Apply xiao-reported origin (NED) to INAV NEU->NED-converted positions
        bb_win["pos_n_rel"] = bb_win["pos_n"] - origin[0]
        bb_win["pos_e_rel"] = bb_win["pos_e"] - origin[1]
        bb_win["pos_d_rel"] = bb_win["pos_d"] - origin[2]
        bb_win["vel_mag"] = np.sqrt(bb_win["vel_n"] ** 2 + bb_win["vel_e"] ** 2 + bb_win["vel_d"] ** 2)

        fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
        colors = ["#d62728", "#2ca02c", "#1f77b4", "#9467bd"]

        # Position
        axes[0].grid(True, linestyle="--", linewidth=0.4)
        # Origin-offset positions (INAV adjusted by virtual origin)
        axes[0].plot(bb_win["t_rel"], bb_win["pos_n_rel"], color=colors[0], linewidth=1.0, label="INAV N (rel)")
        axes[0].plot(bb_win["t_rel"], bb_win["pos_e_rel"], color=colors[1], linewidth=1.0, label="INAV E (rel)")
        axes[0].plot(bb_win["t_rel"], bb_win["pos_d_rel"], color=colors[2], linewidth=1.0, label="INAV D (rel)")
        if not dbg_df.empty:
            axes[0].scatter(dbg_df["t_rel"], dbg_df["pos_n_rel"], s=12, color=colors[0], marker="o", label="xiao N")
            axes[0].scatter(dbg_df["t_rel"], dbg_df["pos_e_rel"], s=12, color=colors[1], marker="o", label="xiao E")
            axes[0].scatter(dbg_df["t_rel"], dbg_df["pos_d_rel"], s=12, color=colors[2], marker="o", label="xiao D")
        axes[0].axvline(0, color="k", linestyle="--", linewidth=0.8)
        axes[0].set_ylabel("Pos (m)")
        axes[0].legend(loc="upper right", fontsize="xx-small", ncol=2)

        # Velocity
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

        # Quaternion
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

        # RC
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

        plt.tight_layout()
        plt.savefig(ROOT / f"span{idx}_overview.png", dpi=150)
        plt.close()

    print("Per-span overview plots written to", ROOT)


def main():
    spans = parse_xiao_log()
    bb = load_blackbox()
    bb = align_blackbox_time(bb, spans)
    print("Detected spans:")
    for s in spans:
        duration = (s.end_ms or s.start_ms) - s.start_ms
        print(f"  {s.label}: {s.start_ms}–{s.end_ms} ms ({duration/1000:.2f}s), evals={len(s.evals)}")

    print("\nRC correlation (xiao setRcData -> INAV rcData nearest sample):")
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

    print("\nVelocity deltas (INAV - Xiao, nearest sample):")
    for s in spans:
        v = velocity_summary(s, bb)
        if not v:
            continue
        vn, ve, vd = v["vel_n"], v["vel_e"], v["vel_d"]
        print(
            f"- {s.label}: dVn mean={vn['mean']:.3f} |mean|={vn['mean_abs']:.3f} "
            f"dVe mean={ve['mean']:.3f} |mean|={ve['mean_abs']:.3f} "
            f"dVd mean={vd['mean']:.3f} |mean|={vd['mean_abs']:.3f}"
        )

    # Generate sensor + RC overlays
    generate_overview_plots(spans, bb)


if __name__ == "__main__":
    main()
