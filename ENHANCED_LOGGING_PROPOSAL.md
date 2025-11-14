# Enhanced Logging Proposal for Frame Convention Debugging

## Current data.dat Format (from autoc.cc:483-537)

### Columns:
```
Pth:Step: Time Idx totDist pathX pathY pathZ X Y Z dr dp dy relVel roll pitch power distP angleP movEffP
```

### Current Fields:
| Column | Variable | Source | Meaning |
|--------|----------|--------|---------|
| Pth | i | Loop counter | Path iteration number |
| Step | simulation_steps | Step counter | Simulation step number |
| Time | getSimTimeMsec() | AircraftState | Simulation time in milliseconds |
| Idx | pathIndex | Path index | Current waypoint index |
| totDist | path[].distanceFromStart | Path | Cumulative path distance |
| pathX/Y/Z | path[].start[] | Path | Target waypoint position (NED meters) |
| X/Y/Z | getPosition()[] | AircraftState | Aircraft position (NED meters) |
| dr | euler[2] (yaw) | Calculated | **Yaw angle (radians)** |
| dp | euler[1] (pitch) | Calculated | **Pitch angle (radians)** |
| dy | euler[0] (roll) | Calculated | **Roll angle (radians)** |
| relVel | getRelVel() | AircraftState | Relative velocity magnitude (m/s) |
| roll | getRollCommand() | AircraftState | Roll command (-1 to 1) |
| pitch | getPitchCommand() | AircraftState | Pitch command (-1 to 1) |
| power | getThrottleCommand() | AircraftState | Throttle command (-1 to 1) |
| distP | distanceFromGoal | Calculated | Distance from goal |
| angleP | angle_rad | Calculated | Angle to goal |
| movEffP | movement_efficiency | Calculated | Movement efficiency metric |

### Euler Angle Calculation (autoc.cc:488-514):
```cpp
Eigen::Matrix3d rotMatrix = stepAircraftState.getOrientation().toRotationMatrix();

euler[0] = atan2(rotMatrix(2, 1), rotMatrix(2, 2)); // roll (phi)
euler[1] = -asin(rotMatrix(2, 0));                  // pitch (theta) - NOTE THE NEGATIVE!
euler[2] = atan2(rotMatrix(1, 0), rotMatrix(0, 0)); // yaw (psi)
```

**CRITICAL FINDING**: Line 512 has `euler[1] = -asin(rotMatrix(2, 0))`

The **negative sign** on pitch calculation! This means **autoc.cc interprets positive pitch as nose DOWN** in its logging!

---

## Problem Analysis

### Current Issues:
1. **Euler angles are only for logging** - GP doesn't use them
2. **Missing GP operator values** - GETALPHA, GETDTHETA, GETDPHI not logged
3. **Missing quaternion raw values** - Can't verify frame conversions
4. **Missing velocity body frame** - Can't verify GETALPHA calculation
5. **No raw sensor correlation** - Can't compare crrcsim vs xiao-gp sensor inputs

### What GP Actually Uses (from GP operators):
- `GETALPHA`: Angle of attack = `atan2(-vz_body, vx_body)`
- `GETDTHETA`: Pitch angle to target in body frame
- `GETDPHI`: Roll angle to target in body frame
- `GETBETA`: Sideslip angle
- `GETVEL`: Velocity magnitude
- `GETDHOME`: Distance to home

These are the values that matter for flight control!

---

## Proposed Enhanced Format

### Add to data.dat header:
```
Pth:Step: Time Idx totDist pathX pathY pathZ X Y Z dr dp dy relVel roll pitch power distP angleP movEffP qw qx qy qz vx_body vy_body vz_body alpha beta dtheta dphi dhome
```

### New Columns:

| Column | Source | Meaning | Why Important |
|--------|--------|---------|---------------|
| **qw, qx, qy, qz** | `getOrientation()` | Raw quaternion components | Verify frame conventions directly |
| **vx_body** | `orientation.inverse() * velocity` (x) | Body frame velocity X | Used in GETALPHA, GETBETA |
| **vy_body** | `orientation.inverse() * velocity` (y) | Body frame velocity Y | Used in GETBETA |
| **vz_body** | `orientation.inverse() * velocity` (z) | Body frame velocity Z | Used in GETALPHA |
| **alpha** | `atan2(-vz_body, vx_body) * 180/π` | Angle of attack (degrees) | **GP operator GETALPHA** |
| **beta** | `atan2(vy_body, vx_body) * 180/π` | Sideslip angle (degrees) | **GP operator GETBETA** |
| **dtheta** | Computed | Pitch angle to target (degrees) | **GP operator GETDTHETA** |
| **dphi** | Computed | Roll angle to target (degrees) | **GP operator GETDPHI** |
| **dhome** | `(home - position).norm()` | Distance to home (meters) | **GP operator GETDHOME** |

---

## Implementation Plan

### 1. Modify autoc.cc (lines 516-538):

```cpp
// Add after existing euler calculation (line 514)

// Calculate body-frame velocity for GP operators
Eigen::Vector3d velocity_body = stepAircraftState.getOrientation().inverse() *
                                 stepAircraftState.getVelocity();

// Calculate GP operator values (matching GP/autoc/gp_operators.h)
double alpha_deg = atan2(-velocity_body.z(), velocity_body.x()) * 180.0 / M_PI;  // GETALPHA
double beta_deg = atan2(velocity_body.y(), velocity_body.x()) * 180.0 / M_PI;    // GETBETA

// Calculate angle to target in body frame (GETDTHETA, GETDPHI)
Eigen::Vector3d craftToTarget = path.at(pathIndex).start - stepAircraftState.getPosition();
Eigen::Vector3d target_body = stepAircraftState.getOrientation().inverse() * craftToTarget;
double dtheta_deg = atan2(-target_body.z(), target_body.x()) * 180.0 / M_PI;    // GETDTHETA
double dphi_deg = atan2(target_body.y(), target_body.x()) * 180.0 / M_PI;       // GETDPHI

// Distance to home (GETDHOME)
Eigen::Vector3d home(0, 0, SIM_INITIAL_ALTITUDE);
double dhome = (home - stepAircraftState.getPosition()).norm();

// Get raw quaternion
Eigen::Quaterniond q = stepAircraftState.getOrientation();

// Update format string to include new fields
sprintf(outbuf, "%03d:%04d: %06ld %3d % 8.2f% 8.2f % 8.2f % 8.2f % 8.2f % 8.2f % 8.2f % 8.2f %8.2f %8.2f % 8.2f % 8.2f % 8.2f % 8.2f % 8.2f % 8.2f % 8.2f % 7.4f % 7.4f % 7.4f % 7.4f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f % 7.2f\n",
    i, simulation_steps,
    stepAircraftState.getSimTimeMsec(), pathIndex,
    path.at(pathIndex).distanceFromStart,
    path.at(pathIndex).start[0],
    path.at(pathIndex).start[1],
    path.at(pathIndex).start[2],
    stepAircraftState.getPosition()[0],
    stepAircraftState.getPosition()[1],
    stepAircraftState.getPosition()[2],
    euler[2],  // dr (yaw)
    euler[1],  // dp (pitch)
    euler[0],  // dy (roll)
    stepAircraftState.getRelVel(),
    stepAircraftState.getRollCommand(),
    stepAircraftState.getPitchCommand(),
    stepAircraftState.getThrottleCommand(),
    distanceFromGoal,
    angle_rad,
    movement_efficiency,
    // NEW FIELDS:
    q.w(), q.x(), q.y(), q.z(),           // Quaternion
    velocity_body.x(), velocity_body.y(), velocity_body.z(),  // Body velocity
    alpha_deg, beta_deg,                  // GETALPHA, GETBETA
    dtheta_deg, dphi_deg,                 // GETDTHETA, GETDPHI
    dhome                                 // GETDHOME
);
```

### 2. Add Matching Logging to xiao-gp

Extend the "DEBUG GP:" log line in [msplink.cpp:287-295](msplink.cpp#L287-L295) to include ALL these values:

```cpp
// Calculate matching values
Eigen::Vector3d velocity_body = aircraft_state.getOrientation().inverse() * aircraft_state.getVelocity();
double alpha_deg = atan2(-velocity_body.z(), velocity_body.x()) * 180.0 / M_PI;
double beta_deg = atan2(velocity_body.y(), velocity_body.x()) * 180.0 / M_PI;
// ... (dtheta, dphi already calculated as debug_getdtheta, debug_getdphi)

logPrint(INFO, "GP SENSORS: quat=[%.4f,%.4f,%.4f,%.4f] vbody=[%.2f,%.2f,%.2f] alpha=%.2f beta=%.2f dtheta=%.2f dphi=%.2f dhome=%.2f",
         aircraft_state.getOrientation().w(),
         aircraft_state.getOrientation().x(),
         aircraft_state.getOrientation().y(),
         aircraft_state.getOrientation().z(),
         velocity_body.x(), velocity_body.y(), velocity_body.z(),
         alpha_deg, beta_deg,
         debug_getdtheta * 180.0 / M_PI,
         debug_getdphi * 180.0 / M_PI,
         dhome);
```

---

## Analysis Workflow

### Step 1: Generate Enhanced Logs
1. **crrcsim**: Run with enhanced data.dat output
2. **xiao-gp**: Collect flight log with "GP SENSORS:" lines

### Step 2: Compare Side-by-Side
For the same maneuver (e.g., pitch up):

| Field | crrcsim | xiao-gp | Match? | Issue |
|-------|---------|---------|--------|-------|
| qw, qx, qy, qz | ? | ? | ? | Frame convention? |
| vx_body | ? | ? | ? | Velocity transform? |
| alpha | ? | ? | ? | **GETALPHA sign?** |
| dtheta | ? | ? | ? | **GETDTHETA sign?** |
| dphi | ? | ? | ? | **GETDPHI sign?** |

### Step 3: Fix Mismatches
Once we identify which GP operator has opposite signs, we know exactly where the frame convention differs!

---

## Expected Outcome

With these enhanced logs, we can **definitively answer**:
1. Are quaternions in the same frame?
2. Is velocity body-frame calculation correct?
3. Do GP operators (GETALPHA, GETDTHETA, GETDPHI) give same values?
4. Which specific convention is inverted?

This will pinpoint the exact fix needed in either:
- `neuQuaternionToNed()` conversion
- Velocity body-frame calculation
- GP operator sign conventions
