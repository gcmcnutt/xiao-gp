# Frame Convention Analysis: crrcsim vs xiao-gp vs INAV

## Problem Statement
Aircraft veering wildly during flight - GP trained in crrcsim sees different sensor presentation than real aircraft via xiao-gp+INAV.

## Key Finding from Bench Test
**User pitched nose UP, but both INAV and xiao-gp reported pitch as NEGATIVE (-17.1°)**

This indicates a **pitch sign convention mismatch** somewhere in the chain.

---

## 1. INAV Frame Convention

### Position/Velocity (from MSP2_INAV_LOCAL_STATE):
- **Frame**: NEU (North-East-Up)
- **Units**: centimeters, cm/s
- `navPos[0]` = North (cm)
- `navPos[1]` = East (cm)
- `navPos[2]` = **UP** (cm) - positive Z is upward

### Quaternion (from MSP2_INAV_LOCAL_STATE):
- **Frame**: Earth→Body rotation in NEU frame
- **Format**: (w, x, y, z) scaled by 10000 in blackbox
- Represents rotation from NEU earth frame to body frame

### Attitude (Euler angles - for reference only):
- **Pitch convention**: "nose up is NEGATIVE pitch" (according to user observation)
- This is **opposite** of typical aerospace convention
- Blackbox attitude values are in decisidegrees (÷10 for degrees)

**CRITICAL**: INAV's pitch sign may be inverted from standard!

---

## 2. xiao-gp Frame Conversion

### NEU→NED Conversion in [msplink.cpp:74-97](msplink.cpp#L74-L97):

```cpp
// Position/Velocity vector conversion
static Eigen::Vector3d neuVectorToNedMeters(const int32_t vec_cm[3])
{
  double north = static_cast<double>(vec_cm[0]) / 100.0;  // ✅ North unchanged
  double east = static_cast<double>(vec_cm[1]) / 100.0;   // ✅ East unchanged
  double down = -static_cast<double>(vec_cm[2]) / 100.0;  // ✅ UP→DOWN (negate Z)
  return Eigen::Vector3d(north, east, down);
}

// Quaternion conversion NEU→NED
static Eigen::Quaterniond neuQuaternionToNed(const float q[4])
{
  Eigen::Quaterniond q_neu(q[0], q[1], q[2], q[3]);
  q_neu.normalize();

  // NEU→NED: 180° rotation around X-axis (North axis)
  Eigen::Quaterniond q_neu_to_ned(0.0, 1.0, 0.0, 0.0);  // 180° around X

  // Transform: Earth(NED)→Body = (NEU→NED) * Earth(NEU)→Body
  Eigen::Quaterniond q_ned = q_neu_to_ned * q_neu;
  q_ned.normalize();

  return q_ned;
}
```

**Analysis**:
- Position/velocity conversion: ✅ Correct (negate Z)
- Quaternion conversion: ✅ Mathematically correct (180° rotation around North/X axis)

---

## 3. crrcsim Frame Convention

### From [inputdev_autoc.cpp:362-406](~/crsim/crrcsim-0.9.13/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp#L362-L406):

```cpp
// Get native quaternion from EOM01 FDM
EOM01* eom01 = dynamic_cast<EOM01*>(Global::aircraft->getFDM());
Eigen::Quaterniond q = Eigen::Quaterniond(
    eom01->getQuatW(),
    eom01->getQuatX(),
    eom01->getQuatY(),
    eom01->getQuatZ()
);

// Position (in feet, convert to meters)
Eigen::Vector3d p{
    Global::aircraft->getPos().r[0] * FEET_TO_METERS,
    Global::aircraft->getPos().r[1] * FEET_TO_METERS,
    Global::aircraft->getPos().r[2] * FEET_TO_METERS
};

// Velocity
Eigen::Vector3d velocity_vector{
    Global::aircraft->getVel()[0] * FEET_TO_METERS,
    Global::aircraft->getVel()[1] * FEET_TO_METERS,
    Global::aircraft->getVel()[2] * FEET_TO_METERS
};

// Populate AircraftState
aircraftState = {pathIndex, v, velocity_vector, q, p, ...};
```

### Euler Angle Calculation (for logging only):
```cpp
Eigen::Vector3d euler = aircraftState.getOrientation().toRotationMatrix().eulerAngles(2, 1, 0);
double roll_rad = euler[2];   // Roll angle (rotation around X-axis)
double pitch_rad = euler[1];  // Pitch angle (rotation around Y-axis)
```

**Question**: What frame does EOM01 use? NED or NEU or something else?

---

## 4. Bench Test Results (bench7)

### Observations at T=7.0s (nose pitched DOWN by user, but reported as):

| Source | Quaternion | Pitch Angle | Match? |
|--------|-----------|-------------|--------|
| INAV blackbox | qw=-0.038, qx=0.109, qy=0.060, qz=0.991 | -12.7° | ✅ |
| xiao-gp | qw=-0.038, qx=0.108, qy=0.060, qz=0.992 | -12.6° | ✅ |

**Finding**: Quaternions match perfectly (<0.001 error), but:
- User physically pitched nose **UP**
- Both systems report pitch as **NEGATIVE**

This means either:
1. **INAV has inverted pitch convention** (nose up = negative), OR
2. **User's perception was opposite** (actually pitched down), OR
3. **Quaternion→Euler conversion has sign error**

---

## 5. Root Cause Hypothesis

### The quaternions themselves are CORRECT (they match between systems).

### The problem is likely in the **Euler angle interpretation**:

In standard aerospace NED convention:
- **Positive pitch (θ)** = nose up
- Rotation matrix element: `sin(pitch) = 2*(qw*qy - qz*qx)`

For qw=-0.038, qx=0.109, qy=0.060, qz=0.991:
```
sin(pitch) = 2*(-0.038*0.060 - 0.991*0.109)
           = 2*(-0.00228 - 0.108019)
           = 2*(-0.110299)
           = -0.2206
pitch = arcsin(-0.2206) = **-12.7°**
```

So if qx is **positive** (which it is), and the nose is pitched **UP**, then:
- Either the quaternion frame is different than we think, OR
- INAV's quaternion has qx representing **negative pitch** motion

---

## 6. ACTION ITEMS

### A. Verify EOM01/crrcsim Frame Convention
- [ ] Check what frame EOM01 quaternion uses (NED? NEU? Body-fixed?)
- [ ] Verify sign convention: does positive qx = nose up or nose down?
- [ ] Test: Pitch aircraft up in crrcsim, log quaternion qx value

### B. Verify INAV Frame Convention
- [ ] Test on bench: pitch nose UP, record quaternion
- [ ] Expected if standard NED: qx should be positive for nose up
- [ ] Check INAV source code for quaternion calculation

### C. Compare GP Operator Inputs
The GP doesn't use Euler angles directly - it uses:
- `GETALPHA`: angle of attack = atan2(-vz_body, vx_body)
- `GETDTHETA`: angle to target in pitch plane
- `GETDPHI`: angle to target in roll plane

Need to verify these are calculated identically in both systems!

### D. Potential Fix Locations
If quaternion sign convention is inverted:
1. **Option 1**: Invert qx and qy in `neuQuaternionToNed()`
2. **Option 2**: Change NEU→NED rotation (might need different axis)
3. **Option 3**: Fix in GP operator calculations (GETALPHA, etc.)

---

## 7. NEXT STEPS

1. **Run crrcsim with DETAILED_LOGGING** and pitch aircraft up
   - Record quaternion values when nose is clearly up
   - Compare with INAV bench test

2. **Add logging to xiao-gp** for GP operators:
   - Log GETALPHA, GETDTHETA, GETDPHI values
   - Compare with crrcsim during identical maneuvers

3. **Create side-by-side test**:
   - Same path in both simulators
   - Log all sensor inputs to GP
   - Find where they diverge

---

## Conclusion

The quaternions are correlating perfectly (< 1° error), which means the **raw sensor data is correct**.

The problem is likely in **how that quaternion is interpreted** - either:
- Euler angle conversion has wrong signs
- GP operators (GETALPHA, etc.) have sign errors
- Frame assumption mismatch between training and flight

**The pitch sign discrepancy is the smoking gun** - need to trace through both code paths to find where the convention flips.
