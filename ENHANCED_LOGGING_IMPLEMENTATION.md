# Enhanced Logging Implementation Complete

## Changes Made

### 1. GP/autoc/autoc.cc

#### Updated data.dat Header (lines 483, 847):
```
Pth:Step: Time Idx totDist pathX pathY pathZ X Y Z dr dp dy relVel roll pitch power distP angleP movEffP qw qx qy qz vxBody vyBody vzBody alpha beta dtheta dphi dhome
```

#### Added GP Operator Calculations (lines 516-535, 868-887):
```cpp
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
```

#### Updated sprintf Format (lines 538-565, 890-917):
Added 12 new fields to output:
- qw, qx, qy, qz (quaternion components)
- vxBody, vyBody, vzBody (body-frame velocity)
- alpha, beta (GETALPHA, GETBETA in degrees)
- dtheta, dphi (GETDTHETA, GETDPHI in degrees)
- dhome (GETDHOME distance)

---

### 2. xiao-gp/src/msplink.cpp

#### Added GP Operator Evaluations (lines 286-287):
```cpp
double debug_getbeta = evaluateGPOperator(19, pathProvider, aircraft_state, debug_args, 1, 0.0);    // GETBETA opcode = 19
double debug_getdhome = evaluateGPOperator(15, pathProvider, aircraft_state, debug_args, 1, 0.0);   // GETDHOME opcode = 15
```

#### Added Body Velocity and Quaternion (lines 289-293):
```cpp
// Calculate body-frame velocity for detailed logging
Eigen::Vector3d velocity_body = aircraft_state.getOrientation().inverse() * aircraft_state.getVelocity();

// Get raw quaternion
Eigen::Quaterniond q = aircraft_state.getOrientation();
```

#### Added GP SENSORS Log (lines 300-308):
```cpp
logPrint(INFO, "GP SENSORS: quat=[%.4f,%.4f,%.4f,%.4f] vbody=[%.2f,%.2f,%.2f] alpha=%.2f beta=%.2f dtheta=%.2f dphi=%.2f dhome=%.2f",
         q.w(), q.x(), q.y(), q.z(),
         velocity_body.x(), velocity_body.y(), velocity_body.z(),
         debug_getalpha * 180.0 / M_PI,      // Convert radians to degrees
         debug_getbeta * 180.0 / M_PI,
         debug_getdtheta * 180.0 / M_PI,
         debug_getdphi * 180.0 / M_PI,
         debug_getdhome);
```

---

## New Data Fields Explanation

### Quaternion (qw, qx, qy, qz)
- **Purpose**: Raw orientation in quaternion form
- **Why**: Verify frame conventions directly without Euler angle ambiguity
- **Format**: Normalized quaternion components (w, x, y, z)

### Body-Frame Velocity (vxBody, vyBody, vzBody)
- **Purpose**: Velocity in aircraft body frame
- **Why**: Used to calculate GETALPHA and GETBETA
- **Format**: Meters/second in body frame (forward, right, down)
- **Calculation**: `orientation.inverse() * velocity_world`

### GETALPHA (alpha)
- **Purpose**: Angle of attack
- **Formula**: `atan2(-vz_body, vx_body)` in degrees
- **Why**: Critical GP operator - wrong sign causes pitch inversion
- **Expected**: Positive when nose pitched up

### GETBETA (beta)
- **Purpose**: Sideslip angle
- **Formula**: `atan2(vy_body, vx_body)` in degrees
- **Why**: GP operator for lateral control
- **Expected**: Positive when nose pointing right

### GETDTHETA (dtheta)
- **Purpose**: Pitch angle to target in body frame
- **Formula**: `atan2(-target_body_z, target_body_x)` in degrees
- **Why**: GP uses this to compute pitch command
- **Expected**: Positive when target is above nose

### GETDPHI (dphi)
- **Purpose**: Roll angle to target in body frame
- **Formula**: `atan2(target_body_y, target_body_x)` in degrees
- **Why**: GP uses this to compute roll command
- **Expected**: Positive when target is to the right

### GETDHOME (dhome)
- **Purpose**: Distance to home point
- **Formula**: `|home - position|` in meters
- **Why**: GP might use for navigation decisions

---

## Usage Instructions

### 1. Test in crrcsim
```bash
cd ~/crsim/crrcsim-0.9.13
# Run simulation with GP controller
# Output will be in ~/GP/autoc/data.dat
```

### 2. Test on xiao-gp
```bash
# Flash updated xiao-gp code
# Enable autoc during flight
# Collect flight log
# Look for "GP SENSORS:" lines
```

### 3. Compare Logs Side-by-Side

Extract a specific maneuver (e.g., pitch up) and compare:

#### From data.dat:
```
Time=5000ms: qw=0.9990 qx=0.0436 qy=0.0000 qz=0.0000 vxBody=20.00 vyBody=0.00 vzBody=-2.00 alpha=5.71 beta=0.00 dtheta=10.23 dphi=0.00
```

#### From flight log:
```
#00123 5000 4998 i GP SENSORS: quat=[0.9990,0.0436,0.0000,0.0000] vbody=[20.00,0.00,-2.00] alpha=5.71 beta=0.00 dtheta=10.23 dphi=0.00 dhome=50.00
```

### 4. Look for Mismatches

| Field | crrcsim | xiao-gp | Match? | Issue |
|-------|---------|---------|--------|-------|
| qw, qx, qy, qz | Check | Check | ? | Frame convention? |
| vxBody | Check | Check | ? | Velocity transform? |
| alpha | Check | Check | ? | **Sign error?** |
| beta | Check | Check | ? | **Sign error?** |
| dtheta | Check | Check | ? | **Sign error?** |
| dphi | Check | Check | ? | **Sign error?** |

**Any sign mismatch indicates the frame convention problem!**

---

## Expected Outcomes

### If Frame Conventions Match:
- All GP operator values will be identical (within numerical precision)
- Aircraft should track path correctly

### If Frame Conventions Differ:
- One or more GP operators will have opposite signs
- This pinpoints exactly which transformation is wrong:
  - **alpha wrong**: `velocity_body.z()` sign issue
  - **beta wrong**: `velocity_body.y()` sign issue
  - **dtheta wrong**: `target_body.z()` transform issue
  - **dphi wrong**: `target_body.y()` transform issue

### Fixing Mismatches:

Once identified, fix in `xiao-gp/src/msplink.cpp`:

```cpp
// Example: If alpha is inverted
static Eigen::Quaterniond neuQuaternionToNed(const float q[4])
{
  // Modify rotation to match crrcsim frame
  // ... (specific fix depends on which operator is wrong)
}
```

---

## Next Steps

1. ✅ **Rebuild autoc**: `cd ~/GP/autoc && make`
2. ✅ **Rebuild xiao-gp**: Flash updated firmware
3. **Run test**: Pitch aircraft up in both systems
4. **Compare logs**: Find matching timestamps
5. **Analyze**: Look for GP operator sign differences
6. **Fix**: Adjust frame conversion in xiao-gp
7. **Verify**: Re-test until all operators match

The smoking gun will be which GP operator(s) have opposite signs between the two systems!
