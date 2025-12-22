#!/usr/bin/env python3
"""
2D path visualization from xiao flight log
Shows craft position vs rabbit target path to identify offset patterns
"""

import re
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# File paths
XIAO_LOG = Path("/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/fligt24-hb1-flight_log_2025-12-21T18-10-34.txt")

def extract_flight_data(log_path):
    """Extract position and rabbit data from xiao log"""

    # Regex patterns
    input_re = re.compile(
        r"GP Input:.*idx=(\d+).*rabbit=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\].*vec=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\]"
    )
    state_re = re.compile(
        r"GP State:.*pos=\[([-0-9\.]+),([-0-9\.]+),([-0-9\.]+)\].*autoc=Y"
    )
    control_re = re.compile(
        r"GP Control: Switch (enabled|disabled)"
    )
    origin_re = re.compile(
        r"GP Control: Switch enabled - test origin NED=\[([-0-9\.]+), ([-0-9\.]+), ([-0-9\.]+)\]"
    )

    # Data storage
    spans = []
    current_span = None

    with open(log_path, 'r') as f:
        for line in f:
            # Check for control enable with origin
            m = origin_re.search(line)
            if m:
                # Save previous span if exists
                if current_span is not None and len(current_span['craft_pos']) > 0:
                    spans.append(current_span)
                # Start new span with captured origin
                origin_n = float(m.group(1))
                origin_e = float(m.group(2))
                origin_d = float(m.group(3))
                current_span = {
                    'origin': (origin_n, origin_e, origin_d),
                    'craft_pos': [],
                    'rabbit_pos': [],
                    'vec': [],
                    'idx': []
                }
                continue

            # Check for control disable
            m = control_re.search(line)
            if m:
                if m.group(1) == "enabled" and current_span is None:
                    # Fallback if origin regex didn't match
                    current_span = {
                        'origin': (0, 0, 0),
                        'craft_pos': [],
                        'rabbit_pos': [],
                        'vec': [],
                        'idx': []
                    }
                elif m.group(1) == "disabled" and current_span is not None:
                    if len(current_span['craft_pos']) > 0:
                        spans.append(current_span)
                    current_span = None
                continue

            if current_span is None:
                continue

            # Extract GP Input (rabbit position)
            m = input_re.search(line)
            if m:
                idx = int(m.group(1))
                rabbit_n = float(m.group(2))
                rabbit_e = float(m.group(3))
                rabbit_d = float(m.group(4))
                vec_n = float(m.group(5))
                vec_e = float(m.group(6))
                vec_d = float(m.group(7))

                current_span['rabbit_pos'].append((rabbit_n, rabbit_e, rabbit_d))
                current_span['vec'].append((vec_n, vec_e, vec_d))
                current_span['idx'].append(idx)

            # Extract GP State (craft position)
            m = state_re.search(line)
            if m:
                pos_n = float(m.group(1))
                pos_e = float(m.group(2))
                pos_d = float(m.group(3))
                current_span['craft_pos'].append((pos_n, pos_e, pos_d))

    # Append last span if still active
    if current_span is not None:
        spans.append(current_span)

    return spans

def plot_2d_path(spans, output_prefix="path_2d"):
    """Create 2D (N-E) plots for each span"""

    for span_idx, span in enumerate(spans, 1):
        if not span['craft_pos'] or not span['rabbit_pos']:
            print(f"Span {span_idx}: No data, skipping")
            continue

        # Convert to numpy arrays
        # Only use points where we have rabbit data (active GP control)
        rabbit = np.array(span['rabbit_pos'])
        vec = np.array(span['vec'])
        num_rabbit = len(rabbit)

        # Trim craft positions to match rabbit count (GP State continues after GP Input stops)
        craft = np.array(span['craft_pos'][:num_rabbit])
        origin = np.array(span.get('origin', (0, 0, 0)))

        print(f"\nSpan {span_idx}:")
        print(f"  Test origin (NED): [{origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f}]")
        print(f"  (Positions shown in virtual path coordinates, origin should be ~[0, 0, -25])")

        # Calculate offset statistics
        # Match craft position to nearest rabbit point by index
        min_len = min(len(craft), len(rabbit))
        craft_matched = craft[:min_len]
        rabbit_matched = rabbit[:min_len]

        offset_n = craft_matched[:, 0] - rabbit_matched[:, 0]
        offset_e = craft_matched[:, 1] - rabbit_matched[:, 1]
        offset_mag = np.sqrt(offset_n**2 + offset_e**2)

        mean_offset_n = np.mean(offset_n)
        mean_offset_e = np.mean(offset_e)
        mean_offset_mag = np.mean(offset_mag)

        print(f"\nSpan {span_idx} Offset Statistics:")
        print(f"  Points: {len(craft)} craft, {len(rabbit)} rabbit")
        print(f"  Mean offset: N={mean_offset_n:+.2f}m, E={mean_offset_e:+.2f}m")
        print(f"  Mean offset magnitude: {mean_offset_mag:.2f}m")
        print(f"  Offset std dev: N={np.std(offset_n):.2f}m, E={np.std(offset_e):.2f}m")

        # Create figure with single plot
        fig, ax1 = plt.subplots(1, 1, figsize=(10, 10))
        fig.suptitle(f'Flight 24 - Span {span_idx} - 2D Path (Virtual Coordinates)', fontsize=14, fontweight='bold')

        # Plot 1: Target dots, Craft dots, vec arrows
        # Sample points for clarity
        step = max(1, min_len // 30)

        # Plot all target waypoints as green dots
        ax1.scatter(rabbit[:, 1], rabbit[:, 0],
                   color='green', s=80, marker='o', alpha=0.6,
                   label='Target Waypoints', zorder=3, edgecolors='darkgreen', linewidths=1)

        # Plot all craft positions as blue squares
        ax1.scatter(craft[:, 1], craft[:, 0],
                   color='blue', s=80, marker='s', alpha=0.6,
                   label='Craft Position', zorder=4, edgecolors='darkblue', linewidths=1)

        # Draw craft→target vec arrows (sampled)
        # vec is in world (NED) coordinates, arrow from craft pointing to target
        for i in range(0, min_len, step):
            if i < len(vec):
                ax1.arrow(craft[i, 1], craft[i, 0],  # Start at craft E, N
                         vec[i, 1], vec[i, 0],        # Vector direction in E, N
                         head_width=1.0, head_length=0.8,
                         fc='red', ec='red', alpha=0.4, linewidth=1, zorder=2,
                         length_includes_head=True)

        # Mark virtual origin
        ax1.scatter([0], [0], color='black', s=300, marker='x', linewidths=4,
                   label='Virtual Origin (0,0)', zorder=5)

        ax1.set_xlabel('East (m)', fontsize=12)
        ax1.set_ylabel('North (m)', fontsize=12)
        ax1.set_title('Target (green), Craft (blue), Vec arrows (red)', fontsize=13)
        ax1.grid(True, alpha=0.4, linewidth=0.5)
        ax1.legend(loc='best', fontsize=10)
        ax1.axis('equal')

        plt.tight_layout()
        output_file = f"{output_prefix}_span{span_idx}.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"  Saved: {output_file}")
        plt.close()

def plot_all_spans_combined(spans, output_file="path_2d_all_spans.png"):
    """Create combined plot showing all spans overlaid"""

    fig, ax = plt.subplots(1, 1, figsize=(12, 12))
    fig.suptitle('Flight 24 - All Spans - 2D Path Comparison', fontsize=14, fontweight='bold')

    colors_rabbit = ['green', 'lime', 'darkgreen']
    colors_craft = ['blue', 'cyan', 'navy']

    for span_idx, span in enumerate(spans, 1):
        if not span['craft_pos'] or not span['rabbit_pos']:
            continue

        craft = np.array(span['craft_pos'])
        rabbit = np.array(span['rabbit_pos'])

        # Plot rabbit path (target)
        ax.plot(rabbit[:, 1], rabbit[:, 0],
                color=colors_rabbit[span_idx-1], linewidth=2,
                label=f'Span {span_idx} Target', alpha=0.6, linestyle='--')

        # Plot craft path (actual)
        ax.plot(craft[:, 1], craft[:, 0],
                color=colors_craft[span_idx-1], linewidth=2,
                label=f'Span {span_idx} Actual', alpha=0.8)

        # Mark start points
        ax.scatter(rabbit[0, 1], rabbit[0, 0],
                  color=colors_rabbit[span_idx-1], s=100, marker='o',
                  edgecolors='black', linewidths=1.5, zorder=5)
        ax.scatter(craft[0, 1], craft[0, 0],
                  color=colors_craft[span_idx-1], s=100, marker='s',
                  edgecolors='black', linewidths=1.5, zorder=5)

    ax.set_xlabel('East (m)', fontsize=12)
    ax.set_ylabel('North (m)', fontsize=12)
    ax.set_title('All Test Spans (Dashed=Target, Solid=Actual)', fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=10)
    ax.axis('equal')

    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nSaved combined plot: {output_file}")
    plt.close()

def main():
    print("=" * 80)
    print("Flight 24 - 2D Path Analysis")
    print("=" * 80)

    spans = extract_flight_data(XIAO_LOG)
    print(f"\nExtracted {len(spans)} test spans from xiao log")

    # Plot individual spans
    plot_2d_path(spans, output_prefix="path_2d")

    # Plot all spans combined
    plot_all_spans_combined(spans)

    print("\n" + "=" * 80)
    print("Analysis complete!")
    print("=" * 80)

if __name__ == "__main__":
    main()
