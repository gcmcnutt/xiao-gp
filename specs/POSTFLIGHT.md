# Analyze log files from post-flight data collection
Here we describe how to analyze the log files generated during post-flight data collection. This includes steps to extract relevant data, visualize flight parameters, and identify any anomalies that may have occurred during the flight.

## data sources
Two log files are generated during post-flight data collection.  They are located in directory: /mnt/c/Users/gmcnutt/OneDrive/Documents/Navigation/HB1-orange-configs/  The two files are:
- flightXX-blackbox_log_[datetime].TXT: This file is generated from inav blackbox data using the ~/blackbox-tools code. INAV uses **NED** (North-East-Down) coordinate frame with body→earth quaternions. The xiao-gp msplink.cpp applies a conjugate transformation to convert to earth→body quaternions matching the GP contract. See ~/GP/autoc/specs/COORDINATE_CONVENTIONS.md for details. There are a few key fields to focus on: 
  - 'time (us)' for correlating time to ~/xiao-gp logs, rcData[], quaternion[], navPos[], navVel[].
  - xiao logfile examples include: 
   #00000130 0056689 0057367 i GP Control: Switch enabled - path origin NED=[182.31, 11.43, -77.67] - starting flight test
   #00000131 0056691 0057367 i DEBUG GP: world_vec=[-1.0,-0.0,0.0] body_vec=[0.9,0.5,-0.0] theta=0.504 phi=1.525 alpha=-0.072 dtarget=-0.535
   #00000132 0056692 0057367 i GP SENSORS: quat=[0.2361,-0.0279,0.1447,0.9605] vbody=[11.12,12.61,0.80] alpha=-4.12 beta=48.59 dtheta=28.88 dphi=87.38 dhome=190.11
   #00000133 0056693 0057367 i DEBUG RABBIT: idx=1, elapsed=1ms, target_y=11.4, craft_y=11.4, dist=1.0
   #00000134 0056703 0057367 i GP Eval: target=[181.3,11.4,-77.7] idx=1 setRcData=[1665,1420,1232]
   #00000135 0056708 0057367 i GP State: pos=[182.3,11.4,-77.7] vel=[-15.7,-5.5,2.8] att=[15.5,7.0,206.7] quat=[0.236,-0.028,0.145,0.960] relvel=16.8 armed=Y fs=N servo=Y autoc=Y rabbit=Y

   - analyze the code to determine how these fields are generated.  See ~/xiao-gp/src/msplink.cpp and ~/xiao-gp/src/gpcontrol.cpp for details.

- flightXX-flight_log_[datetime].TXT: This is logfile from ~/xiao-gp code. Several log lines are in this file which can time correlate to blackbox data.
  - each of the above fields should correlate to blackbox data
  - SENSORS after unit conversion should be fine.
  - pay attention to the polarity of the various alpha, beta, dtheta, dphi, dhome values.  These can be tricky.
  - RABBIT line will help correlate to where on the path we should be, something relatable to Eval target line

### timestamp alignment cheatsheet
- Every xiao log line is `#<msgId> <xiaoMillis> <inavSampleMillis> <level> <message>`. The second column is the local `millis()` when `logPrint()` ran, while the third column is the INAV sample timestamp in milliseconds copied straight from `MSP2_INAV_LOCAL_STATE.timestamp_us / 1000` (`src/msplink.cpp:408-414`). The formatter in `logPrint()` writes both fields before the log body (`src/util.cpp:48-90`).
- In the INAV blackbox CSV (`flight20.csv`), the `time (us)` column is the exact value that feeds `timestamp_us`. Divide that value by 1000 (or multiply the xiao third column by 1000) to correlate the two logs sample-for-sample.
- Offsets between the two clocks are expected. Pick any distinctive xiao event (e.g. `GP Control: Switch enabled`) and match its third column to the nearest `time (us)` entry to get the absolute alignment for that flight. After that, delta times in either log overlay directly.
- The first column in the xiao log (`#00000130` in the example above) is the monotonic message ID produced by the flash logger; it is useful for spotting drops but does not appear in INAV data.

Important: attempt to correlate ALL xiao debug values by looking at the code in both xiao and in inav blackbox tools.  There are many translations and unit conversions to get right.

## goals
For these early runs we want to understand that we can correlate inav data to xiao-gp data. And then from there we want to ensure that ~/inav coordinate systems and units of measure are properly translated by ~/xiao-gp to be 'lined up'.  Once we are convinced of this (see ~/GP/autoc/COORINDATE_CONVENTIONS.md for more details on coordinate systems), we can then move on to analyzing flight dynamics and sensor translation errors.

The primary goal is to understand a live flight and why it isn't quite the same as a training/simulation run run using ~/GP/autoc and the ~/crsim/crrcsim-0.9.13 simulator.  There are many translations in here to believe in in particular, the crrcsim/src/mod-inputdev/inputdev-autoc translation for the crrcsim and the ~/xiao-gp/src/msplink.cpp code does a similar translation.

At some point we should able to understand that basic sensor and output data agrees between the simulation runs like recorded by autoc in files like ~/GP/autoc/data.dat based on ~/GP/autoc/autoc-eval.ini configurations.

When the do agree, then we are left with flight dynamics differences between simulation and real world flight.  These can be due to unmodeled aerodynamics, wind, or other factors.

## connections
- logfile correlation: use 'time (us)' field in blackbox log to correlate to 'time' field in xiao-gp log which comes from the msp get time calls in msplink.cpp
- coordinate systems: **Both INAV and xiao-gp/GP use NED** (North-East-Down) coordinates. INAV sends body→earth quaternions via MSP which msplink.cpp converts to earth→body via conjugate transformation. Blackbox logs contain raw INAV body→earth quaternions. See ~/GP/autoc/specs/COORDINATE_CONVENTIONS.md for complete details.
- rc command outputs from xiao wind up as rcData[] fields in inav blackbox log. the actual servo fields rcServo[] are outputs to command servos after running through a mixer, etc.  rcData[0] should line up with roll, rcData[1] should line up with pitch, and rcData[2] should be throttle.  Will need to dig into inav to ensure the commands are turning into the conventional commands.

### INAV RC columns to capture
- `rcData[0..3]` in the blackbox CSV are roll, pitch, throttle, yaw (1000–2000 µs). `rcCommand[0..3]` are the FC’s internal commands after rates/expo. These are the columns to line up against xiao `setRcData`.
- Servo outputs show up as `servo[0]`, `servo[1]` (per-logframe mixer output) and are useful to confirm mixer behavior but not for direct GP correlation.
- If you need override status, `mspOverrideFlags` is the bitfield that marks MSP RC override activity; include it when extracting slices around GP Control spans.

### quick decode note (flight21)
From `~/GP/autoc`, we decoded the flight21 INAV blackbox with:
```
../../blackbox-tools/obj/blackbox_decode --index 0 --stdout "/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/flight21-blackbox_log_2025-12-16_171249.TXT" > flight21.csv
```
The resulting CSV was used for time correlation against the xiao flight log.

## analysis workflow

### Step 1: Decode blackbox log
Decode the INAV blackbox .TXT file to CSV using blackbox_decode:
```bash
cd ~/GP/autoc
~/xiao-gp/postflight/blackbox-tools/blackbox_decode --index 0 --stdout \
  "/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/flight24-blackbox_log_2025-12-21_100239.TXT" \
  > flight24.csv
```

### Step 2: Create flight directory and analysis scripts
```bash
mkdir -p ~/xiao-gp/postflight/flight24
cd ~/xiao-gp/postflight/flight24
```

Copy and adapt analysis scripts from a previous flight (e.g., flight23):
- `analyze_flight24.py` - Main analysis script that:
  - Parses GP Control spans from xiao log
  - Correlates GP Output RC commands to INAV rcData
  - Applies origin offset convention
  - Generates per-span overlays for position/velocity/quaternion and RC
  - Outputs timing and rcData deltas to spot latency issues
- `control_response_flight24.py` - Control→state response plots:
  - Sim: commands vs roll/pitch/speed from data.dat
  - Xiao: RC commands vs roll/pitch/speed per span

Update file paths in both scripts to point to:
- BLACKBOX_CSV: `/home/gmcnutt/GP/autoc/flight24.csv`
- XIAO_LOG: path in `/mnt/c/Users/gcmcn/OneDrive/Documents/Navigation/HB1-orange-configs/`
- SIM_DATA: `/home/gmcnutt/GP/autoc/data.dat`

### Step 3: Run analysis scripts
```bash
cd ~/xiao-gp/postflight/flight24
python3 analyze_flight24.py      # Generates span{1,2,3}_overview.png
python3 control_response_flight24.py  # Generates control_response_*.png
```

### Step 4: Review outputs
The scripts generate:
- **span{N}_overview.png**: Per-span plots showing:
  - Position (N, E, D) overlaid with path target
  - Velocity magnitude
  - Quaternion components (qw, qx, qy, qz)
  - RC commands (roll, pitch, throttle)

- **control_response_sim.png**: Simulator control→state response
  - Commands (roll_cmd, pitch_cmd, throttle) on left axis
  - Resulting roll/pitch attitude and speed on right axis

- **control_response_xiao_span{N}.png**: Per-span xiao control→state response
  - RC commands (1000-2000µs) on left axis
  - Resulting roll/pitch attitude and speed on right axis

Console output shows:
- Detected test spans with timing and sample counts
- RC correlation statistics (lag, deltas between commanded and actual)
- GP input parity check (recomputed vs logged sensor values)

### Analysis goals
1. **Correlation verification**: Ensure xiao debug values correlate to INAV blackbox data after all unit conversions and coordinate transformations.

2. **Coordinate system validation**: Verify INAV data (NED, body→earth quaternions) is correctly transformed by xiao-gp (conjugate to earth→body) to match GP training data contract. See ~/GP/autoc/specs/COORDINATE_CONVENTIONS.md.

3. **Sensor parity**: Confirm GP sensor inputs (alpha, beta, dtheta, dphi, dhome, relvel) match between logged values and recomputed values from raw state data.

4. **Control response**: Compare GP output in flight vs simulation:
   - Early flights may show sensor polarity issues, quaternion errors, unit conversion problems
   - Later flights should show dynamics differences (unmodeled aero, wind, etc.)

5. **Simulation alignment**: Compare with ~/GP/autoc/data.dat to isolate whether issues are in GP eval, sensor translation, or flight dynamics.

The ultimate goal is a robust sim-to-real pipeline for generating flight control for aircraft models with variations.  
