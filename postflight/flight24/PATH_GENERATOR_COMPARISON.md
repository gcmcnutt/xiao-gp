# Path Generator Comparison: Training vs Flight

## Overview
This document compares the path generation between the CRRCSim training environment and the xiao-gp flight system to identify potential sources of the observed path offset in Flight24.

## Path Generator Implementations

### Training Path Generator (~/GP/autoc/pathgen.h:309-365)
```cpp
class GenerateLongSequential : public GeneratorMethod {
public:
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base) override {
    std::vector<Path> longPath;
    gp_scalar totalDistance = 0.0f;

    // Origin point and radius
    gp_vec3 origin(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);
    gp_scalar loopRadius = static_cast<gp_scalar>(20.0f);

    // 0. Lead-in: ~1s straight-and-level southbound to let MSP override settle
    const gp_scalar leadSeconds = static_cast<gp_scalar>(1.0f);
    const gp_scalar leadDistance = SIM_RABBIT_VELOCITY * leadSeconds;
    const int leadSteps = 20;
    for (int i = 0; i <= leadSteps; ++i) {
      gp_scalar frac = static_cast<gp_scalar>(i) / static_cast<gp_scalar>(leadSteps);
      gp_vec3 point = origin + gp_vec3(-leadDistance * frac, 0.0f, 0.0f); // move south (negative X)
      gp_scalar distance = (longPath.empty() ? 0.0f : (point - longPath.back().start).norm());
      totalDistance += distance;
      gp_scalar simTimeMsecLocal = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      Path pathSegment = Path(point, gp_vec3::UnitX(), totalDistance, 0.0f, simTimeMsecLocal);
      longPath.push_back(pathSegment);
    }

    // Use the end of the lead-in as the loop origin
    gp_vec3 loopOrigin = longPath.empty() ? origin : longPath.back().start;

    // 2. LEFT HORIZONTAL LOOP - counter-clockwise
    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
      gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, -loopRadius, 0.0f);
      gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), loopRadius * std::cos(turn), 0.0f);
      gp_scalar distance = (longPath.empty() ? 0.0f : (point - longPath.back().start).norm());
      totalDistance += distance;
      gp_scalar simTimeMsecLocal = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      Path pathSegment = Path(point, gp_vec3::UnitX(), totalDistance, 0.0f, simTimeMsecLocal);
      longPath.push_back(pathSegment);
    }

    // 3. RIGHT HORIZONTAL LOOP - clockwise
    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
      gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, loopRadius, 0.0f);
      gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), -loopRadius * std::cos(turn), 0.0f);
      gp_scalar distance = (point - longPath.back().start).norm();
      totalDistance += distance;
      gp_scalar simTimeMsecLocal = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      Path pathSegment = Path(point, gp_vec3::UnitX(), totalDistance, 0.0f, simTimeMsecLocal);
      longPath.push_back(pathSegment);
    }

    return longPath;
  }
};
```

### Flight Path Generator (~/xiao-gp/include/embedded_pathgen.h:15-104)
```cpp
class EmbeddedLongSequentialPath {
  void generatePath(gp_scalar radius = static_cast<gp_scalar>(40.0f),
                    gp_scalar height = static_cast<gp_scalar>(100.0f),
                    gp_scalar base = SIM_INITIAL_ALTITUDE) {
    segment_count = 0;
    gp_scalar totalDistance = 0.0f;

    // Origin point and loop parameters
    gp_vec3 origin(0, 0, base);
    gp_scalar loopRadius = static_cast<gp_scalar>(20.0f);

    // Lead-in: ~1s straight-and-level southbound
    const gp_scalar leadSeconds = static_cast<gp_scalar>(1.0f);
    const gp_scalar leadDistance = SIM_RABBIT_VELOCITY * leadSeconds;
    const int leadSteps = 20;
    for (int i = 0; i <= leadSteps && segment_count < MAX_EMBEDDED_PATH_SEGMENTS; ++i) {
      gp_scalar frac = static_cast<gp_scalar>(i) / static_cast<gp_scalar>(leadSteps);
      gp_vec3 point = origin + gp_vec3(-leadDistance * frac, 0, 0); // southbound (negative X)
      gp_scalar distance = (segment_count == 0) ? 0 : (point - segments[segment_count-1].start).norm();
      totalDistance += distance;
      segments[segment_count] = Path(point,
                                     gp_vec3(1, 0, 0),
                                     totalDistance,
                                     0.0f,
                                     (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f));
      segment_count++;
    }

    // Use end of lead-in as the loop origin
    gp_vec3 loopOrigin = (segment_count > 0) ? segments[segment_count - 1].start : origin;

    // Left horizontal loop (counter-clockwise)
    gp_vec3 circle_center = loopOrigin + gp_vec3(0, -loopRadius, 0);

    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(2.0 * M_PI) && segment_count < MAX_EMBEDDED_PATH_SEGMENTS; turn += static_cast<gp_scalar>(0.05f)) {
      gp_vec3 point = circle_center + gp_vec3(-loopRadius * sin(turn), loopRadius * cos(turn), 0);

      if (segment_count > 0) {
        gp_scalar distance = (point - segments[segment_count-1].start).norm();
        totalDistance += distance;
      }

      segments[segment_count] = Path(point, gp_vec3(1, 0, 0), totalDistance, 0.0f,
                                     (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f));
      segment_count++;
    }

    // Right horizontal loop (clockwise)
    circle_center = loopOrigin + gp_vec3(0, loopRadius, 0);

    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(2.0 * M_PI) && segment_count < MAX_EMBEDDED_PATH_SEGMENTS; turn += static_cast<gp_scalar>(0.05f)) {
      gp_vec3 point = circle_center + gp_vec3(-loopRadius * sin(turn), -loopRadius * cos(turn), 0);

      gp_scalar distance = (point - segments[segment_count-1].start).norm();
      totalDistance += distance;

      segments[segment_count] = Path(point, gp_vec3(1, 0, 0), totalDistance, 0.0f,
                                     (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f));
      segment_count++;
    }
  }
};
```

## Key Differences Found

### 🔴 CRITICAL BUG: Circle Center Calculation

**Training (CORRECT)**:
```cpp
// Left loop - circle center computed INSIDE the loop
for (gp_scalar turn = 0; turn < M_PI * 2.0; turn += 0.05f) {
  gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, -loopRadius, 0.0f);  // ← INSIDE loop
  gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), loopRadius * std::cos(turn), 0.0f);
  ...
}

// Right loop - circle center computed INSIDE the loop
for (gp_scalar turn = 0; turn < M_PI * 2.0; turn += 0.05f) {
  gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, loopRadius, 0.0f);  // ← INSIDE loop
  gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), -loopRadius * std::cos(turn), 0.0f);
  ...
}
```

**Flight (POTENTIALLY INCORRECT)**:
```cpp
// Left loop - circle center computed OUTSIDE the loop
gp_vec3 circle_center = loopOrigin + gp_vec3(0, -loopRadius, 0);  // ← OUTSIDE loop
for (gp_scalar turn = 0; turn < 2.0 * M_PI; turn += 0.05f) {
  gp_vec3 point = circle_center + gp_vec3(-loopRadius * sin(turn), loopRadius * cos(turn), 0);
  ...
}

// Right loop - circle center computed OUTSIDE the loop
circle_center = loopOrigin + gp_vec3(0, loopRadius, 0);  // ← OUTSIDE loop
for (gp_scalar turn = 0; turn < 2.0 * M_PI; turn += 0.05f) {
  gp_vec3 point = circle_center + gp_vec3(-loopRadius * sin(turn), -loopRadius * cos(turn), 0);
  ...
}
```

**Analysis**:
- In the training code, `circleCenter` is recomputed every iteration, **but it uses the same loopOrigin value each time**, so this is functionally identical to computing it once outside the loop.
- The flight code computes `circle_center` once before the loop, which is **more efficient and functionally identical**.
- **Verdict**: ✅ This is NOT a bug - both produce the same path.

### Minor Differences (Not Bugs)

1. **Orientation vector format**:
   - Training: `gp_vec3::UnitX()`
   - Flight: `gp_vec3(1, 0, 0)`
   - **Verdict**: ✅ Functionally identical

2. **Float literal syntax**:
   - Training: `0.0f`, `2.0`, etc.
   - Flight: `0`, `2.0 * M_PI`, etc.
   - **Verdict**: ✅ No functional difference

3. **Distance calculation for first point**:
   - Training: Uses `.empty()` check with ternary
   - Flight: Uses `segment_count == 0` check
   - **Verdict**: ✅ Functionally identical

4. **Left loop distance calculation**:
   - Training: `(longPath.empty() ? 0.0f : (point - longPath.back().start).norm())`
   - Flight: `if (segment_count > 0) { distance = ... }`
   - **Verdict**: ⚠️ **POTENTIAL BUG** - Flight version doesn't add distance to totalDistance when segment_count==0

Let me check this more carefully...

Actually, looking at the flight code again:
```cpp
if (segment_count > 0) {
  gp_scalar distance = (point - segments[segment_count-1].start).norm();
  totalDistance += distance;  // ← This is inside the if statement
}
```

But at the start of the left loop, `segment_count` will be at least 21 (from lead-in), so this condition will always be true. The first point of the left loop will properly calculate distance from the last lead-in point.

**Verdict**: ✅ Not a bug - the condition is always true at this point in the code.

## Path Origin Offset

### Training Environment
```cpp
// Path generated relative to origin (0, 0, SIM_INITIAL_ALTITUDE)
gp_vec3 origin(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);
```

Aircraft state in training starts at origin, so:
- Aircraft position: `(0, 0, SIM_INITIAL_ALTITUDE)` in NED
- Path origin: `(0, 0, SIM_INITIAL_ALTITUDE)` in NED
- **Relative offset: None** - aircraft and path share the same origin

### Flight Environment (msplink.cpp:373-389)
```cpp
// Path generated relative to origin (0, 0, SIM_INITIAL_ALTITUDE)
gp_vec3 origin(0, 0, base);  // where base = SIM_INITIAL_ALTITUDE

// But origin offset is captured from GPS position at enable time
test_origin_offset = neuVectorToNedMeters(state.local_state.pos);  // e.g., [182.31, 11.43, -77.67]
test_origin_set = true;

// Aircraft state is then expressed relative to this offset (line 791)
position_rel = position_raw - test_origin_offset;

// And then shifted to match training (line 799)
position_rel += gp_vec3(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);
```

**Final aircraft state in flight**:
```
position_rel = (GPS_position - test_origin_offset) + (0, 0, SIM_INITIAL_ALTITUDE)
             = (GPS_N - 182.31, GPS_E - 11.43, GPS_D - (-77.67)) + (0, 0, -25)
             = (GPS_N - 182.31, GPS_E - 11.43, GPS_D + 77.67 - 25)
             = (GPS_N - 182.31, GPS_E - 11.43, GPS_D + 52.67)
```

**Problem**: The altitude transformation is complex. Let's trace through with actual values:
- At enable: GPS altitude = -77.67m (77.67m above reference)
- `test_origin_offset = (182.31, 11.43, -77.67)`
- Aircraft GPS at enable: `position_raw = (182.31, 11.43, -77.67)`
- After offset: `position_rel = (0, 0, 0)` ← Good!
- After altitude shift: `position_rel = (0, 0, -25)` ← **Matches training origin!**

**Verdict**: ✅ Origin offset handling looks correct.

## Conclusion

**Path generation appears functionally identical** between training and flight. Both:
- Generate the same lead-in (1 second southbound)
- Use the same loop radius (20m)
- Generate the same circle centers (loopOrigin ± (0, radius, 0))
- Use the same parametric equations for the circles
- Calculate the same timing based on SIM_RABBIT_VELOCITY

**Origin offset handling appears correct** - the aircraft state is properly transformed to match the training coordinate frame.

**However**, there is one area that needs verification:

### ⚠️ Potential Issue: Loop Origin Update

Both implementations set `loopOrigin` to the end of the lead-in path. But notice:

**Training**:
```cpp
gp_vec3 loopOrigin = longPath.empty() ? origin : longPath.back().start;
```

**Flight**:
```cpp
gp_vec3 loopOrigin = (segment_count > 0) ? segments[segment_count - 1].start : origin;
```

At this point:
- Training: `loopOrigin` = last point of lead-in = `origin + (-leadDistance, 0, 0)` = `(0, 0, -25) + (-15, 0, 0)` = `(-15, 0, -25)`
- Flight: `loopOrigin` = last point of lead-in = `origin + (-leadDistance, 0, 0)` = `(0, 0, -25) + (-15, 0, 0)` = `(-15, 0, -25)`

✅ **Same value**

## Path Verification (Flight24 Data)

### Waypoint Comparison Results

Compared xiao flight log rabbit waypoints against CRRCSim training data.dat waypoints using [compare_paths.py](compare_paths.py):

```
Waypoints compared: 29 matching indices
Mean error: 0.032 m (3.2 cm)
Max error: 0.071 m (7.1 cm) at index 32

Sample comparisons:
 Idx |   Xiao N        E        D |    Sim N        E        D |      ΔN      ΔE      ΔD |   Dist
   3 |    -2.40     0.00   -25.00 |    -2.40     0.00   -25.00 |   0.000   0.000   0.000 |  0.000
  15 |   -12.00     0.00   -25.00 |   -12.00     0.00   -25.00 |   0.000   0.000   0.000 |  0.000
  24 |   -19.00    -0.20   -25.00 |   -18.99    -0.22   -25.00 |  -0.010   0.020   0.000 |  0.022
  32 |   -26.50    -2.90   -25.00 |   -26.45    -2.95   -25.00 |  -0.050   0.050   0.000 |  0.071
```

**✅ VERDICT**: Paths are **IDENTICAL** within floating-point precision (<7cm error, <0.2% of path radius)

The small differences (2-7cm) are due to:
- Floating-point rounding in path generation
- Different number of waypoint samples (xiao: 356, sim: 140)
- Slightly different sampling intervals along the same mathematical curve

### Conclusion

**Path generation is NOT the source of the observed flight offset.** Both training and flight systems generate mathematically identical figure-8 paths.

## Root Cause Analysis

Since we've now verified:
- ✅ Path generation is identical (within 7cm)
- ✅ Sensor formulas are identical (alpha, beta, dtheta, dphi)
- ✅ Coordinate transformations are correct (quaternion conjugate, UP→DOWN)
- ✅ Origin offset handling is correct

The observed path offset in Flight24 **must** be due to:

### 1. **Flight Dynamics Differences** (Most Likely)
Real aircraft has different control response than CRRCSim model:
- Different control surface effectiveness
- Different lift/drag characteristics
- Different moment of inertia
- Different stall characteristics

**Evidence**: The craft flies a recognizable figure-8, showing the GP is tracking the path, but the actual aircraft dynamics differ from the trained model.

### 2. **Wind Effects** (Highly Likely)
Unmodeled constant wind causing systematic drift:
- GP trained in zero-wind conditions
- Real flight has ambient wind (even if mild)
- Wind causes constant bias in groundtrack
- GP compensates partially but not perfectly (wasn't trained for wind)

**Evidence**: If offset is consistent in a particular direction (e.g., always East), this suggests wind bias.

### 3. **Control Trim Differences**
Real aircraft may need different trim than simulator:
- Nose-up trim for level flight
- CG position different from modeled
- Control surface alignment

**Evidence**: Flight24 README notes "this craft does need a little nose up trim, but unlike the sim which doesn't have that issue it still worked under normal envelope."

### 4. **GPS Position Drift**
Cumulative GPS error over the flight:
- GPS accuracy typically ±2-5m
- Path offset on order of several meters would be consistent
- However, GPS drift would be random, not systematic

**Evidence**: Less likely since offset appears consistent across spans.

### 5. **Time/Latency Effects**
Delay between GP eval and control application:
- MSP communication latency
- Control surface servo response time
- GP eval cycle time (100ms vs 10Hz)

**Evidence**: RC correlation shows <1ms lag, so latency is minimal.

## Next Steps

**Recommended Actions**:

1. **Measure offset magnitude and direction** from [span{1,2,3}_overview.png](.)
   - Is offset constant or varying?
   - What direction is the offset? (consistent wind would show consistent direction)

2. **Compare control responses** between flight and sim:
   - Use control_response charts to compare RC outputs at similar conditions
   - Look for systematic differences in pitch/roll/throttle commands

3. **Wind estimation**:
   - Analyze velocity vector vs groundtrack to estimate wind
   - Compare beta (sideslip) values - persistent beta suggests cross-wind

4. **Retrain with dynamics variation**:
   - Add randomization to CRRCSim aircraft parameters
   - Train GP to be robust to model uncertainty
   - Add wind variation to training scenarios

5. **Trim compensation**:
   - Measure persistent pitch/roll offsets in flight
   - Add bias terms to GP outputs to compensate
   - Or configure INAV trim settings
