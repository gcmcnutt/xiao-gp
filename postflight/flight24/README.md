# Flight 24 Analysis (2025-12-21)

## Overview
Flight 24 is the first flight showing "intention in the correct direction of flying the virtual path" after implementing the quaternion conjugate fix (2025-12-20).

## Key Coordinate Fix
**CRITICAL**: This is the first flight using the corrected quaternion transformation:
- INAV sends **body→earth** quaternions in **NED** frame (NOT NEU as previously thought)
- `msplink.cpp` now applies **conjugate transformation** to convert to earth→body
- See `~/GP/autoc/specs/COORDINATE_CONVENTIONS.md` for full details

## Data Sources
- **Blackbox CSV**: `/home/gmcnutt/GP/autoc/flight24.csv`
- **Xiao Log**: `/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/fligt24-hb1-flight_log_2025-12-21T18-10-34.txt`
- **Sim Data**: `/home/gmcnutt/GP/autoc/data.dat`

## Test Spans
Three autonomous control spans detected:

| Span | Duration | Samples | Origin (NED) |
|------|----------|---------|--------------|
| test1 | 12.12s | 118 | (165.68, 8.56, -77.66) |
| test2 | 12.23s | 119 | (192.88, -0.21, -82.94) |
| test3 | 12.20s | 119 | (205.94, -11.49, -108.95) |

## Analysis Results

### RC Correlation (Xiao → INAV)
Shows GP commands correctly reaching INAV with reasonable latency:
- Mean lag: -123µs to 838µs (sub-millisecond)
- Forward window (50ms) lag: ~10ms average
- RC deltas: 44-58µs mean (good tracking)

### GP Input Parity
Excellent agreement between logged and recomputed sensor values:
- Alpha/Beta: <0.04° mean error, <0.13° max
- Dtheta/Dphi: <0.14° mean error, <1.75° max
- Dhome: <0.001m mean error

### Quaternion Behavior
Charts show **expected mirroring of qz and qy** on all three flights, confirming proper NED/FRD coordinate system with earth→body quaternions.

## Generated Charts

### Overview Plots (`span{1,2,3}_overview.png`)
Per-span visualization showing:
- Position (N, E, D) vs path target
- Velocity magnitude
- Quaternion components (qw, qx, qy, qz) - **shows proper mirroring**
- RC commands (roll, pitch, throttle)

### Control Response Plots
- `control_response_sim.png`: Simulator control→state response baseline
- `control_response_xiao_span{1,2,3}.png`: Per-span flight control→state response

## Significance
This flight demonstrates:
1. ✅ Correct quaternion transformation (conjugate fix working)
2. ✅ Proper NED coordinate system throughout pipeline
3. ✅ Good sensor value agreement (GP inputs match expected values)
4. ✅ **First intentional path-following behavior observed**

The quaternion fix has resolved the fundamental coordinate mismatch between INAV and the GP training data!
