# Path Coordinate System Refactoring

## Problem Statement

Currently, path generation has inconsistent coordinate system handling between desktop (autoc) and embedded (xiao-gp):

- **Desktop (autoc)**: Paths generated with `base` parameter (e.g., -25m in NED), path points at absolute altitude
- **Embedded (xiao-gp)**: Paths generated with current GPS altitude (e.g., 946.64m), resulting in 25m vertical offset from craft position

This creates a **25m altitude mismatch** where the embedded craft (at virtual 0,0,0) is chasing paths at virtual (0,0,946.64) instead of (0,0,0).

## Root Cause

Path generation mixes two concerns:
1. **Path geometry** (shapes, maneuvers)
2. **Coordinate frame offsets** (where to place the path in world space)

Current implementation bakes the offset into path generation via the `base` parameter, making it impossible to share pure geometry between desktop and embedded.

## Proposed Solution

### 1. Pure Path Generation (Geometry Only)

All path generators produce paths in a **canonical coordinate frame**:
- Origin: `(0, 0, 0)`
- Entry point: `(0, 0, 0)` heading south (-X direction)
- All maneuvers relative to this origin

**Benefits**:
- Path geometry is identical between desktop and embedded
- Easy to verify correctness (no mental math for coordinate transforms)
- Single source of truth for each maneuver

### 2. Coordinate Transform on Output

Apply coordinate offsets **after** path generation, **before** adding to output vector:

**Desktop (autoc)**:
```cpp
// Generate path in canonical frame (0,0,0)
GenerateAeroStandard gen;
std::vector<Path> canonical_path = gen.method(pathIndex, radius, height, 0.0f, seed);

// Apply offset for NED frame (-25m altitude)
gp_vec3 offset(0.0f, 0.0f, -25.0f);
for (auto& segment : canonical_path) {
  segment.start += offset;
}
```

**Embedded (xiao-gp)**:
```cpp
// Generate path in canonical frame (0,0,0)
path_generator.generatePath(pathIndex, 0.0f, EMBEDDED_PATH_SEED);

// No offset needed - craft is already at virtual (0,0,0)
// OR apply current GPS altitude offset if needed
```

## Implementation Plan

### Phase 1: Update Desktop pathgen.h

**Changes to `GenerateAeroStandard::method()`**:
1. Remove `base` parameter from entry point: `gp_vec3 entryPoint(0.0f, 0.0f, 0.0f);`
2. Remove `base` from all helper methods (addStraightSegment, addHorizontalTurn, etc.)
3. Keep `base` parameter in signature for backward compatibility, but **ignore it**
4. Document that `base` is deprecated

**Changes to helper methods**:
```cpp
void addStraightSegment(std::vector<Path>& path, const gp_vec3& start, const gp_vec3& direction,
                        gp_scalar distance, gp_scalar& totalDistance) {
  // ... generates points starting from 'start', no altitude offset
}
```

**Changes to `generateSmoothPaths()`**:
```cpp
std::vector<std::vector<Path>> generateSmoothPaths(char* method, int numPaths, gp_scalar radius, gp_scalar height, unsigned int baseSeed) {
  // ... existing code ...

  // Generate paths in canonical frame
  std::vector<Path> canonicalPath = generator->method(pathIndex, radius, height, 0.0f, seed);

  // Apply NED offset for desktop simulation
  gp_vec3 offset(0.0f, 0.0f, SIM_INITIAL_ALTITUDE);  // -25m in NED
  std::vector<Path> offsetPath;
  for (const auto& segment : canonicalPath) {
    Path offsetSegment = segment;
    offsetSegment.start += offset;
    offsetPath.push_back(offsetSegment);
  }

  paths.push_back(offsetPath);
}
```

### Phase 2: Update Embedded pathgen_selector.h

**Changes to `EmbeddedPathSelector::generatePath()`**:
1. Change signature: `void generatePath(int pathIndex, gp_scalar zOffset = 0.0f, unsigned int seed = EMBEDDED_PATH_SEED)`
2. Remove `base` from entry point: `gp_vec3 entryPoint(0.0f, 0.0f, 0.0f);`
3. Remove `base` from all switch cases
4. Apply `zOffset` when creating Path segments (if needed)

**Example**:
```cpp
void generatePath(int pathIndex, gp_scalar zOffset = 0.0f, unsigned int seed = EMBEDDED_PATH_SEED) {
  segment_count = 0;
  was_truncated = false;
  gp_scalar totalDistance = 0.0f;

  AeroStandardPathType pathType = static_cast<AeroStandardPathType>(pathIndex % static_cast<int>(AERO_END_MARKER));
  gp_vec3 entryPoint(0.0f, 0.0f, 0.0f);  // Canonical origin

  // Generate path geometry in canonical frame
  switch(pathType) {
    case StraightAndLevel: {
      // Generate at (0,0,0) origin
      addStraightSegment(entryPoint, gp_vec3(-1.0f, 0.0f, 0.0f), 40.0f, totalDistance);
      // ...
    }
  }

  // Apply z-offset if needed (currently 0 for xiao-gp)
  if (zOffset != 0.0f) {
    for (int i = 0; i < segment_count; i++) {
      segments[i].start[2] += zOffset;
    }
  }
}
```

### Phase 3: Update xiao-gp caller

**Changes to `/home/gmcnutt/xiao-gp/src/msplink.cpp`**:
```cpp
// OLD (line 407):
path_generator.generatePath(pathIndex, base_altitude, EMBEDDED_PATH_SEED);

// NEW:
path_generator.generatePath(pathIndex, 0.0f, EMBEDDED_PATH_SEED);  // No z-offset, craft at (0,0,0)
```

**Update logging**:
```cpp
// OLD:
logPrint(INFO, "Path armed: %d=%s, %d/%d segments, alt=%.2fm, seed=%u, time=%.1fms",
         pathIndex, pathNames[pathIndex], (int)flight_path.size(), MAX_EMBEDDED_PATH_SEGMENTS,
         base_altitude, EMBEDDED_PATH_SEED, generation_duration_us / 1000.0f);

// NEW:
logPrint(INFO, "Path armed: %d=%s, %d/%d segments, origin=(0,0,0), seed=%u, time=%.1fms",
         pathIndex, pathNames[pathIndex], (int)flight_path.size(), MAX_EMBEDDED_PATH_SEGMENTS,
         EMBEDDED_PATH_SEED, generation_duration_us / 1000.0f);
```

## Validation

### Desktop (autoc)
- Paths start at (0, 0, -25) after offset applied
- Existing eval results unchanged
- Flight tests match previous behavior

### Embedded (xiao-gp)
- Paths start at (0, 0, 0) with craft at virtual (0, 0, 0)
- **Fixes 25m altitude offset bug**
- GP rabbit vectors now correctly point from craft to nearby path points

## Migration Path

1. **Phase 1**: ✅ COMPLETE - Updated desktop pathgen.h (generate at 0,0,0, apply offset in generateSmoothPaths)
2. **Phase 2**: ✅ COMPLETE - Updated embedded pathgen_selector.h documentation
3. **Phase 3**: ✅ COMPLETE - Updated xiao-gp caller (pass 0.0f for base)
4. **Validation**: Test both desktop and embedded implementations
5. **Cleanup**: Remove deprecated `base` parameters from method signatures (future work)

## Future Work

Once stabilized, consolidate pathgen.h and embedded_pathgen_selector.h into single file with:
- `#ifdef EMBEDDED_BUILD` for storage (static array vs vector)
- Identical path generation logic
- Shared coordinate system (0,0,0 canonical frame)
