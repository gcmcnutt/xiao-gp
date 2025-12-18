# Analyze log files from post-flight data collection
Here we describe how to analyze the log files generated during post-flight data collection. This includes steps to extract relevant data, visualize flight parameters, and identify any anomalies that may have occurred during the flight.

## data sources
Two log files are generated during post-flight data collection.  They are located in directory: /mnt/c/Users/gmcnutt/OneDrive/Documents/Navigation/HB1-orange-configs/  The two files are:
- flightXX-blackbox_log_[datetime].TXT: This file is generated from inav blackbox data using the ~/blackbox-tools code. Keep in mind the coordinate conventions for inav is NEU.  There are a few key fields to focus on: 
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
- coordinate systems: inav uses NEU, xiao-gp uses NED.  See ~/GP/autoc/COORDINATE_CONVENTIONS.md for more details.
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

## steps
1. Generate analytics programs to do basic parsing of the two log files.
2. These files typically relate to more than one test inside a single flight. in INAV we should find places where MSPRCOVERRIDE are set. This is when autoc takes over.  Similar log events are seen in the ~/xiao-gp flight logs as marked by lines like 'GP Control: Switch enabled'.
3. Be convinced that inav input is correctly represented as sensor input to the generated GP. Recall the flow for generating the GP is to do a training run using ~/GP/autoc with a configuration file like autoc.ini. Later this archived GP is evaluated using autoc-eval.ini.  After all looks good, this GP is translated to source code for xiao-gp in its generated directory.
4. Once convinced of correct input, then analyze the output of the GP in flight versus the output of the GP in simulation.  Look for differences and try to understand why.

Early runs I expect both sensor and control output to be not quite right.  Sensor or control force polarity issues. Quaternion orientation incorrect. Units of measure, etc.

Also we can occasionally enhance the ~/crsim/crrcsim-0.9.13/models/hb1.xml.  The ultimate goal is a sensible 'sim to real' pipline for genreating real world flight control for models with slight variations.

5. We can also compare unusual controls to how the GP eval runs as seen in data.dat files from ~/GP/autoc.  This can help isolate if the issue is in the GP eval or in the sensor translation or flight dynamics.  
