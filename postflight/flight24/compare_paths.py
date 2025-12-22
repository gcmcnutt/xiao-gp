#!/usr/bin/env python3
"""Compare xiao flight path waypoints against training simulator path"""

import re
import math
from pathlib import Path

# File paths
XIAO_LOG = Path("/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/fligt24-hb1-flight_log_2025-12-21T18-10-34.txt")
SIM_DATA = Path("/home/gmcnutt/GP/autoc/data.dat")

# Extract rabbit waypoints from xiao log
def extract_xiao_waypoints(log_path):
    """Extract rabbit target waypoints from xiao log"""
    waypoints = []
    rabbit_re = re.compile(r"GP Input:.*idx=(\d+).*rabbit=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\]")

    with open(log_path, 'r') as f:
        for line in f:
            m = rabbit_re.search(line)
            if m:
                idx = int(m.group(1))
                n = float(m.group(2))
                e = float(m.group(3))
                d = float(m.group(4))
                waypoints.append((idx, n, e, d))

    return waypoints

# Extract path waypoints from simulator data.dat
def extract_sim_waypoints(data_path):
    """Extract path waypoints from simulator data.dat

    Format: scenario:index: time idx N E D ...
    Example: 000:0001: 000117   1     1.00    -0.79     0.00   -25.00 ...
    """
    waypoints = []

    with open(data_path, 'r') as f:
        for line in f:
            if line.startswith('000:'):  # Scenario 0 only
                parts = line.split()
                if len(parts) >= 7:
                    # scenario:index: time idx N E D ...
                    idx = int(parts[2])
                    n = float(parts[4])
                    e = float(parts[5])
                    d = float(parts[6])
                    waypoints.append((idx, n, e, d))

    return waypoints

def main():
    print("=" * 80)
    print("Path Waypoint Comparison: Xiao Flight vs CRRCSim Training")
    print("=" * 80)

    # Extract waypoints
    xiao_wpts = extract_xiao_waypoints(XIAO_LOG)
    sim_wpts = extract_sim_waypoints(SIM_DATA)

    print(f"\nXiao flight waypoints: {len(xiao_wpts)}")
    print(f"Simulator waypoints: {len(sim_wpts)}")

    # Create sim waypoint lookup by index
    sim_by_idx = {idx: (n, e, d) for idx, n, e, d in sim_wpts}

    # Compare waypoints at matching indices
    print("\n" + "=" * 80)
    print("Waypoint Comparison (first 50 matches)")
    print("=" * 80)
    print(f"{'Idx':>4} | {'Xiao N':>8} {'E':>8} {'D':>8} | {'Sim N':>8} {'E':>8} {'D':>8} | {'ΔN':>7} {'ΔE':>7} {'ΔD':>7} | {'Dist':>6}")
    print("-" * 120)

    max_error = 0.0
    max_error_idx = 0
    total_error = 0.0
    count = 0

    for idx, xn, xe, xd in xiao_wpts[:50]:
        if idx in sim_by_idx:
            sn, se, sd = sim_by_idx[idx]
            dn = xn - sn
            de = xe - se
            dd = xd - sd
            dist = math.sqrt(dn*dn + de*de + dd*dd)

            print(f"{idx:4d} | {xn:8.2f} {xe:8.2f} {xd:8.2f} | {sn:8.2f} {se:8.2f} {sd:8.2f} | {dn:7.3f} {de:7.3f} {dd:7.3f} | {dist:6.3f}")

            if dist > max_error:
                max_error = dist
                max_error_idx = idx
            total_error += dist
            count += 1

    if count > 0:
        print("\n" + "=" * 80)
        print("Statistics")
        print("=" * 80)
        print(f"Waypoints compared: {count}")
        print(f"Mean error: {total_error / count:.4f} m")
        print(f"Max error: {max_error:.4f} m at index {max_error_idx}")

        if max_error < 0.001:
            print("\n✅ VERDICT: Paths are IDENTICAL (errors < 1mm)")
        elif max_error < 0.01:
            print("\n✅ VERDICT: Paths match within rounding error (< 1cm)")
        else:
            print(f"\n⚠️  VERDICT: Paths differ by up to {max_error:.3f}m")

    # Show full lead-in comparison
    print("\n" + "=" * 80)
    print("Lead-in Segment Detail (idx 0-20)")
    print("=" * 80)
    print(f"{'Idx':>4} | {'Xiao Waypoint (N, E, D)':>30} | {'Sim Waypoint (N, E, D)':>30} | {'Error':>8}")
    print("-" * 100)

    for idx, xn, xe, xd in xiao_wpts[:21]:
        if idx in sim_by_idx:
            sn, se, sd = sim_by_idx[idx]
            dn = xn - sn
            de = xe - se
            dd = xd - sd
            dist = math.sqrt(dn*dn + de*de + dd*dd)
            print(f"{idx:4d} | ({xn:8.2f}, {xe:8.2f}, {xd:8.2f}) | ({sn:8.2f}, {se:8.2f}, {sd:8.2f}) | {dist:7.4f}m")

if __name__ == "__main__":
    main()
