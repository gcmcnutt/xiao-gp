# GP Sensor Angle Analysis - Flight24 vs CRRCSim Training

## Overview
This document analyzes the four GP sensor angles (alpha, beta, dtheta, dphi) to verify their polarity and implementation consistency between the xiao-gp flight system and CRRCSim training environment.

## Sensor Definitions

| Sensor | Name | Definition | Formula | Polarity Convention |
|--------|------|------------|---------|---------------------|
| **alpha** | Angle of Attack | Angle between velocity vector and body X-axis (forward) in XZ plane | `atan2(-vz_body, vx_body)` | **Positive**: Velocity below nose (nose pitched up relative to velocity)<br>**Negative**: Velocity above nose (nose pitched down relative to velocity) |
| **beta** | Sideslip Angle | Angle between velocity vector and body XZ plane | `atan2(vy_body, vx_body)` | **Positive**: Velocity to the right (aircraft drifting right)<br>**Negative**: Velocity to the left (aircraft drifting left) |
| **dtheta** | Pitch Error to Target | Pitch angle needed to point nose at next path waypoint | `atan2(-tz_body, tx_body)` | **Positive**: Target above nose (need to pitch up)<br>**Negative**: Target below nose (need to pitch down) |
| **dphi** | Roll Error to Target | Roll angle needed to align wings with target direction | `atan2(ty_body, -tz_body)` | **Positive**: Target to the right (need to roll right)<br>**Negative**: Target to the left (need to roll left) |

### Common Coordinate Conventions
- **World frame**: NED (North-East-Down)
- **Body frame**: FRD (Forward-Right-Down)
  - X-axis: Forward through nose
  - Y-axis: Right through right wing
  - Z-axis: Down through belly
- **Quaternion**: earth→body transformation
- **Velocity**: `v_body = q.inverse() * v_world`
- **Target vector**: `t_body = q.inverse() * (target_pos - craft_pos)`

## Implementation Verification

### Source Code Locations

#### CRRCSim/GP Training (~/GP/autoc/gp_evaluator_portable.cc)

```cpp
// Lines 226-232: GETALPHA
case GETALPHA: {
    gp_vec3 velocity_body = aircraftState.getOrientation().inverse() * aircraftState.getVelocity();
    result = fastAtan2(-velocity_body.z(), velocity_body.x());
    break;
}

// Lines 234-240: GETBETA
case GETBETA: {
    gp_vec3 velocity_body = aircraftState.getOrientation().inverse() * aircraftState.getVelocity();
    result = fastAtan2(velocity_body.y(), velocity_body.x());
    break;
}

// Lines 282-293: GETDTHETA
gp_scalar executeGetDTheta(PathProvider& pathProvider, AircraftState& aircraftState, gp_scalar arg) {
    int idx = getPathIndex(pathProvider, aircraftState, arg);
    gp_vec3 craftToTarget = pathProvider.getPath(idx).start - aircraftState.getPosition();
    gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;
    return fastAtan2(-target_local.z(), target_local.x());
}

// Lines 267-280: GETDPHI
gp_scalar executeGetDPhi(PathProvider& pathProvider, AircraftState& aircraftState, gp_scalar arg) {
    int idx = getPathIndex(pathProvider, aircraftState, arg);
    gp_vec3 craftToTarget = pathProvider.getPath(idx).start - aircraftState.getPosition();
    gp_vec3 target_local = aircraftState.getOrientation().inverse() * craftToTarget;
    gp_vec3 projectedVector(0.0f, target_local.y(), target_local.z());
    return fastAtan2(projectedVector.y(), -projectedVector.z());
}
```

#### Xiao-GP Flight System (~/xiao-gp/src/msplink.cpp)

```cpp
// Lines 222-237: Calls identical GP evaluator functions
gp_scalar debug_getdtheta = evaluateGPOperator(GETDTHETA, pathProvider, aircraft_state, nullptr, 0, 0.0f);
gp_scalar debug_getdphi = evaluateGPOperator(GETDPHI, pathProvider, aircraft_state, nullptr, 0, 0.0f);
gp_scalar debug_getalpha = evaluateGPOperator(GETALPHA, pathProvider, aircraft_state, nullptr, 0, 0.0f);
gp_scalar debug_getbeta = evaluateGPOperator(GETBETA, pathProvider, aircraft_state, nullptr, 0, 0.0f);
```

**Conclusion**: Xiao-GP uses **identical** sensor computation functions from gp_evaluator_portable.cc. No polarity or formula differences.

## Sample Data Comparison

### Flight24 Xiao Log Sample (First Test Span)
```
Time(ms)  Idx  Rabbit_NED              CraftToTarget_NED        alpha    beta    dtheta   dphi    dhome   relvel
323752    1    [-0.8,  0.0, -25.0]     [-0.8,  0.0,  0.0]      -0.048   0.279   -0.323  -2.501   0.000   13.82
323864    3    [-2.4,  0.0, -25.0]     [-1.2,  0.6,  0.5]      -0.013   0.225   -0.767  -2.257   1.403   13.68
323972    5    [-4.0,  0.0, -25.0]     [-1.7,  1.2,  1.0]       0.001   0.169   -1.063  -2.270   2.783   13.53
324069    7    [-5.6,  0.0, -25.0]     [-2.2,  1.8,  1.6]      -0.006   0.159   -1.229  -2.288   4.146   13.37
324165    9    [-7.2,  0.0, -25.0]     [-2.7,  2.3,  2.1]      -0.018   0.177   -1.314  -2.300   5.485   13.20
324271    11   [-8.8,  0.0, -25.0]     [-3.3,  2.9,  2.7]      -0.032   0.193   -1.358  -2.300   6.791   13.07
324372    13   [-10.4, 0.0, -25.0]     [-3.9,  3.5,  3.3]      -0.027   0.213   -1.400  -2.327   8.106   12.90
324474    15   [-12.0, 0.0, -25.0]     [-4.5,  4.0,  4.0]      -0.017   0.220   -1.433  -2.345   9.398   12.68
324572    17   [-13.6, 0.0, -25.0]     [-5.2,  4.5,  4.6]       0.001   0.200   -1.448  -2.327  10.567   12.45
324673    19   [-15.2, 0.0, -25.0]     [-6.1,  5.0,  5.3]       0.053   0.215   -1.036  -1.859  11.627   11.98
324777    22   [-17.0,-0.0, -25.0]     [-7.3,  5.3,  5.9]       0.034   0.241    1.020  -1.174  12.572   11.50  ← dtheta sign flip
324878    24   [-19.0,-0.2, -25.0]     [-8.7,  5.5,  6.4]      -0.093   0.158    1.291  -0.501  13.422   11.06
324988    25   [-20.0,-0.4, -25.0]     [-8.9,  6.0,  6.9]      -0.117  -0.006    1.374   0.108  14.591   11.02  ← dphi sign flip
325078    27   [-21.9,-0.9, -25.0]     [-9.9,  6.3,  7.4]      -0.053  -0.128    1.304   0.501  15.830   11.24
325176    28   [-22.9,-1.2, -25.0]     [-9.9,  6.7,  7.8]      -0.084  -0.220    1.188   0.640  17.047   11.48
```

### Observations from Flight24 Data

1. **Alpha** (angle of attack):
   - Range: -0.339 to +0.053 rad (-19° to +3°)
   - Mostly negative (velocity above nose = nose pitched down relative to velocity)
   - Reasonable for cruise flight

2. **Beta** (sideslip):
   - Range: -0.281 to +0.279 rad (-16° to +16°)
   - Crosses zero as aircraft turns through the figure-8
   - Sign changes match expected left/right drift

3. **Dtheta** (pitch to target):
   - Range: -1.448 to +1.374 rad (-83° to +79°)
   - **Sign flip at idx=22**: Target transitions from below nose to above nose
   - Makes sense as aircraft approaches and passes the turning point

4. **Dphi** (roll to target):
   - Range: -2.501 to +0.640 rad (-143° to +37°)
   - **Sign flip at idx=25**: Target transitions from left to right
   - Large negative values early = target far to the left

## Path Offset Analysis

### Observed Behavior
Flight24 shows the craft flying a figure-8 pattern as expected, but **offset from the virtual training path**. The craft appears to consistently fly to one side of the intended path.

### Possible Root Causes

#### 1. Path Origin Convention
- **Xiao logs**: `GP Control: Switch enabled - path origin NED=[182.31, 11.43, -77.67]`
- **Convention**: All path waypoints are relative to this origin
- **Check**: Verify analyze_flight24.py correctly applies origin offset when overlaying path

#### 2. Sensor Polarity Issues
All four sensor formulas match between flight and training, but verify:
- ✅ **Alpha**: `atan2(-vz, vx)` - Matches training
- ✅ **Beta**: `atan2(vy, vx)` - Matches training
- ✅ **Dtheta**: `atan2(-tz, tx)` - Matches training
- ✅ **Dphi**: `atan2(ty, -tz)` - Matches training

#### 3. Quaternion Transformation
- ✅ INAV sends body→earth quaternions (confirmed via bench test)
- ✅ msplink.cpp applies conjugate to convert to earth→body
- ✅ GP expects earth→body quaternions
- ✅ Transformation verified correct

#### 4. Velocity Vector Issues
**CRITICAL CHECK NEEDED**: How is velocity computed?
- **Training (CRRCSim)**: `aircraftState.getVelocity()` returns NED world-frame velocity in m/s
- **Flight (INAV)**: `navVel[]` in cm/s, third component is **climb rate (UP)**, not DOWN

From POSTFLIGHT.md line 136:
```python
df["vel_d"] = -df["vel_u"] / 100.0  # navVel[2] is climb rate (up), negate for NED down
```

**Action**: Verify msplink.cpp correctly negates navVel[2] when creating velocity vector.

#### 5. Position Vector Issues
From POSTFLIGHT.md line 134:
```python
df["pos_d"] = -df["pos_u"] / 100.0  # navPos[2] is altitude (up), negate for NED down
```

**Action**: Verify msplink.cpp correctly negates navPos[2] when creating position vector.

#### 6. Path Waypoint Coordinate Frame
**Check**: Are the path waypoints defined in the same NED frame as the aircraft state?
- Path origin is set at GP Control enable time
- All waypoints are offsets from this origin
- Verify path generator uses NED coordinates

#### 7. Wind/Dynamics Differences
If all coordinate systems are correct, offset may be due to:
- Unmodeled wind (GP trained in calm conditions)
- Different flight dynamics (real aircraft vs CRRCSim model)
- Control surface trim differences

## Recommended Verification Steps

### Step 1: Verify Velocity Vector in msplink.cpp ✅
**VERIFIED CORRECT** (msplink.cpp:131-138)

```cpp
static gp_vec3 neuVectorToNedMeters(const int32_t vec_cm[3])
{
  const gp_scalar inv100 = static_cast<gp_scalar>(0.01f);
  gp_scalar north = static_cast<gp_scalar>(vec_cm[0]) * inv100;
  gp_scalar east = static_cast<gp_scalar>(vec_cm[1]) * inv100;
  gp_scalar down = -static_cast<gp_scalar>(vec_cm[2]) * inv100;  // ✅ Negates UP to get DOWN
  return gp_vec3(north, east, down);
}
```

Used at line 783: `velocity = neuVectorToNedMeters(state.local_state.vel);`

### Step 2: Verify Position Vector in msplink.cpp ✅
**VERIFIED CORRECT** (uses same `neuVectorToNedMeters` function)

Used at line 782: `position_raw = neuVectorToNedMeters(state.local_state.pos);`

The Z-component negation is correct for both position and velocity.

### Step 3: Add Debug Logging
Add logging in msplink.cpp to compare:
- Raw INAV navVel vs computed velocity_body
- Raw INAV navPos vs computed craft-to-target vector
- Computed alpha/beta vs expected values for known flight conditions

### Step 4: Compare Sensor Values Flight vs Sim
Extract similar flight segments from:
- Flight24 control_response charts
- CRRCSim data.dat
Compare alpha, beta, dtheta, dphi ranges and behavior

### Step 5: Check Path Offset Magnitude
Measure the lateral offset between flown path and intended path:
- If offset is ~constant: Likely a coordinate frame issue
- If offset varies: Likely a dynamics/wind issue
- If offset is in specific direction: Check for sign error in specific axis

## Sensor Polarity Quick Reference

| Sensor | Positive Value Means | Negative Value Means | Example Flight24 Range |
|--------|---------------------|----------------------|----------------------|
| **alpha** | Velocity below nose<br>(nose up relative to airflow) | Velocity above nose<br>(nose down relative to airflow) | -0.339 to +0.053 rad<br>(-19° to +3°) |
| **beta** | Velocity to right<br>(aircraft drifting right) | Velocity to left<br>(aircraft drifting left) | -0.281 to +0.279 rad<br>(-16° to +16°) |
| **dtheta** | Target above nose<br>(pitch up to acquire) | Target below nose<br>(pitch down to acquire) | -1.448 to +1.374 rad<br>(-83° to +79°) |
| **dphi** | Target to right<br>(roll right to acquire) | Target to left<br>(roll left to acquire) | -2.501 to +0.640 rad<br>(-143° to +37°) |

## Summary

✅ **Verified Correct**:
- All four sensor formulas match exactly between flight and training
- Quaternion transformation (conjugate) is correct (msplink.cpp:145)
- Both systems use NED world frame and FRD body frame
- ✅ **Position vector UP→DOWN conversion** (msplink.cpp:136, line 782)
- ✅ **Velocity vector UP→DOWN conversion** (msplink.cpp:136, line 783)

⚠️ **Still Under Investigation**:
- Path offset magnitude and direction
- Path origin offset application in analysis scripts
- Control trim differences (real aircraft vs sim)
- Possible unmodeled wind effects

🔍 **Next Actions**:
1. ✅ ~~Check msplink.cpp velocity and position vector construction~~ - VERIFIED CORRECT
2. Measure actual lateral offset magnitude from span{1,2,3}_overview.png charts
3. Compare sensor value ranges (alpha, beta, dtheta, dphi) between flight24 and sim data.dat
4. Check if offset is constant or varies with flight phase
5. Consider adding wind bias parameter to GP training
