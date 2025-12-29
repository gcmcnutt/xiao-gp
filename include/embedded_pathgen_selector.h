#ifndef EMBEDDED_PATHGEN_SELECTOR_H
#define EMBEDDED_PATHGEN_SELECTOR_H

// Embedded path selector for xiao-gp
// Implements 6 AeroStandard paths from GP/autoc/pathgen.h
// Uses desktop-compatible std::mt19937 for SeededRandomB reproducibility

#include <GP/autoc/aircraft_state.h>
#include <vector>
#include <math.h>
#include <random>  // For std::mt19937 (path 5 only)

// Maximum segments based on measured density (1m straight, 0.05 rad turns):
// Path 0: 205, Path 1: 249, Path 2: 269, Path 3: 13, Path 4: 357, Path 5: 239
#define MAX_EMBEDDED_PATH_SEGMENTS 400  // Path 4 needs 357, add margin for safety
#define EMBEDDED_PATH_SEED 67890         // Matches autoc.ini RandomPathSeedB
#define NUM_SEGMENTS_PER_PATH 16         // For random path control points

// AeroStandard path types (matches pathgen.h)
enum AeroStandardPathType {
  StraightAndLevel = 0,
  SpiralClimb,
  HorizontalFigureEight,
  FortyFiveDegreeAngledLoop,
  HighPerchSplitS,
  SeededRandomB,
  AERO_END_MARKER  // Total: 6 paths
};

// Cubic interpolation for smooth paths (from pathgen.h)
inline gp_vec3 cubicInterpolate(const gp_vec3& p0, const gp_vec3& p1,
                                const gp_vec3& p2, const gp_vec3& p3, gp_scalar t) {
  gp_scalar t2 = t * t;
  gp_scalar t3 = t2 * t;

  gp_vec3 a = p3 - p2 - p0 + p1;
  gp_vec3 b = p0 - p1 - a;
  gp_vec3 c = p2 - p0;
  gp_vec3 d = p1;

  return a * t3 + b * t2 + c * t + d;
}

// Generate random point in cylinder using mt19937 PRNG (desktop-compatible)
inline gp_vec3 localRandomPointInCylinder(std::mt19937& rng, gp_scalar radius,
                                          gp_scalar height, gp_scalar base) {
  std::uniform_real_distribution<gp_scalar> dist(static_cast<gp_scalar>(0.0),
                                                  static_cast<gp_scalar>(1.0));
  gp_scalar r = radius * std::cbrt(dist(rng));
  gp_scalar theta = dist(rng) * static_cast<gp_scalar>(M_PI * 2.0);
  gp_scalar z = base - dist(rng) * height;
  gp_scalar x = r * std::cos(theta);
  gp_scalar y = r * std::sin(theta);
  return gp_vec3(x, y, z);
}

class EmbeddedPathSelector {
private:
  Path segments[MAX_EMBEDDED_PATH_SEGMENTS];
  int segment_count;
  bool was_truncated;

  // Helper: Add straight segment
  void addStraightSegment(const gp_vec3& start, const gp_vec3& direction,
                         gp_scalar distance, gp_scalar& totalDistance) {
    const gp_scalar step = static_cast<gp_scalar>(1.0f);  // 1m spacing (baseline from horizontal 8 analysis)
    gp_vec3 dir = direction.normalized();

    // Start from step if path is not empty to avoid duplicating last point
    gp_scalar startD = (segment_count == 0) ? static_cast<gp_scalar>(0.0f) : step;

    for (gp_scalar d = startD; d <= distance; d += step) {
      if (segment_count >= MAX_EMBEDDED_PATH_SEGMENTS) {
        was_truncated = true;
        return;
      }

      gp_vec3 point = start + dir * d;
      if (segment_count > 0) {
        gp_scalar segmentDist = (point - segments[segment_count-1].start).norm();
        totalDistance += segmentDist;
      }
      gp_scalar simTimeMsec = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      segments[segment_count++] = Path(point, gp_vec3::UnitX(), totalDistance, static_cast<gp_scalar>(0.0f), simTimeMsec);
    }
  }

  // Helper: Add horizontal turn
  void addHorizontalTurn(const gp_vec3& start, gp_scalar radius,
                        gp_scalar angleRadians, bool clockwise, gp_scalar& totalDistance) {
    const gp_scalar step = static_cast<gp_scalar>(0.05f);  // 0.05 rad (~3°) spacing - baseline from horizontal 8

    // Determine initial heading from last two points if possible
    gp_vec3 heading(static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)); // default south
    if (segment_count >= 2) {
      heading = (segments[segment_count-1].start - segments[segment_count-2].start).normalized();
    }

    // Project heading onto XY plane
    gp_vec3 headingXY(heading[0], heading[1], static_cast<gp_scalar>(0.0f));
    headingXY = headingXY.normalized();

    // Right perpendicular: rotate (x,y) by -90° = (y, -x)
    gp_vec3 rightPerpendicular(-headingXY[1], headingXY[0], static_cast<gp_scalar>(0.0f));

    // Place center
    gp_vec3 center = start + rightPerpendicular * (clockwise ? static_cast<gp_scalar>(1.0f) : static_cast<gp_scalar>(-1.0f)) * radius;

    // Starting angle
    gp_scalar startAngle = std::atan2(start[1] - center[1], start[0] - center[0]);

    // Clockwise means increasing angle when looking down from above
    gp_scalar angleSign = clockwise ? static_cast<gp_scalar>(1.0f) : static_cast<gp_scalar>(-1.0f);

    for (gp_scalar angle = step; angle <= angleRadians; angle += step) {
      if (segment_count >= MAX_EMBEDDED_PATH_SEGMENTS) {
        was_truncated = true;
        return;
      }

      gp_scalar totalAngle = startAngle + angleSign * angle;
      gp_vec3 point(center[0] + radius * std::cos(totalAngle),
                    center[1] + radius * std::sin(totalAngle),
                    start[2]);

      if (segment_count > 0) {
        gp_scalar segmentDist = (point - segments[segment_count-1].start).norm();
        totalDistance += segmentDist;
      }

      gp_scalar simTimeMsec = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      segments[segment_count++] = Path(point, gp_vec3::UnitX(), totalDistance, static_cast<gp_scalar>(0.0f), simTimeMsec);
    }
  }

  // Helper: Add spiral turn (climbing/descending turn)
  void addSpiralTurn(const gp_vec3& start, gp_scalar radius,
                    gp_scalar angleRadians, bool clockwise, gp_scalar totalClimb,
                    gp_scalar& totalDistance) {
    const gp_scalar step = static_cast<gp_scalar>(0.05f);  // 0.05 rad (~3°) spacing - baseline from horizontal 8

    gp_vec3 heading(static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f));
    if (segment_count >= 2) {
      heading = (segments[segment_count-1].start - segments[segment_count-2].start).normalized();
    }

    gp_vec3 headingXY(heading[0], heading[1], static_cast<gp_scalar>(0.0f));
    headingXY = headingXY.normalized();

    gp_vec3 rightPerpendicular(-headingXY[1], headingXY[0], static_cast<gp_scalar>(0.0f));
    gp_vec3 center = start + rightPerpendicular * (clockwise ? static_cast<gp_scalar>(1.0f) : static_cast<gp_scalar>(-1.0f)) * radius;

    gp_scalar startAngle = std::atan2(start[1] - center[1], start[0] - center[0]);
    gp_scalar angleSign = clockwise ? static_cast<gp_scalar>(1.0f) : static_cast<gp_scalar>(-1.0f);

    for (gp_scalar angle = step; angle <= angleRadians; angle += step) {
      if (segment_count >= MAX_EMBEDDED_PATH_SEGMENTS) {
        was_truncated = true;
        return;
      }

      gp_scalar totalAngle = startAngle + angleSign * angle;
      gp_scalar zOffset = (angle / angleRadians) * totalClimb;
      gp_vec3 point(center[0] + radius * std::cos(totalAngle),
                    center[1] + radius * std::sin(totalAngle),
                    start[2] + zOffset);

      if (segment_count > 0) {
        gp_scalar segmentDist = (point - segments[segment_count-1].start).norm();
        totalDistance += segmentDist;
      }

      gp_scalar simTimeMsec = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      segments[segment_count++] = Path(point, gp_vec3::UnitX(), totalDistance, static_cast<gp_scalar>(0.0f), simTimeMsec);
    }
  }

  // Helper: Add horizontal loop
  void addHorizontalLoop(const gp_vec3& loopOrigin, gp_scalar loopRadius,
                        bool clockwise, gp_scalar& totalDistance) {
    const gp_scalar step = static_cast<gp_scalar>(0.05f);  // 0.05 rad (~3°) spacing - baseline from horizontal 8
    gp_scalar sign = clockwise ? static_cast<gp_scalar>(-1.0f) : static_cast<gp_scalar>(1.0f);

    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += step) {
      if (segment_count >= MAX_EMBEDDED_PATH_SEGMENTS) {
        was_truncated = true;
        return;
      }

      gp_vec3 circleCenter = loopOrigin + gp_vec3(static_cast<gp_scalar>(0.0f), sign * loopRadius, static_cast<gp_scalar>(0.0f));
      gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), -sign * loopRadius * std::cos(turn), static_cast<gp_scalar>(0.0f));

      if (segment_count > 0) {
        gp_scalar distance = (point - segments[segment_count-1].start).norm();
        totalDistance += distance;
      }
      gp_scalar simTimeMsec = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      segments[segment_count++] = Path(point, gp_vec3::UnitX(), totalDistance, static_cast<gp_scalar>(0.0f), simTimeMsec);
    }
  }

  // Helper: Add pitch-down loop (Split-S)
  void addPitchDownLoop(const gp_vec3& start, const gp_vec3& heading,
                       gp_scalar loopRadius, gp_scalar& totalDistance) {
    const gp_scalar step = static_cast<gp_scalar>(0.05f);  // 0.05 rad (~3°) spacing - baseline from horizontal 8

    gp_vec3 headingNorm = heading.normalized();
    gp_vec3 center = start + gp_vec3(static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), loopRadius);

    for (gp_scalar angle = step; angle <= static_cast<gp_scalar>(M_PI); angle += step) {
      if (segment_count >= MAX_EMBEDDED_PATH_SEGMENTS) {
        was_truncated = true;
        return;
      }

      gp_scalar horizontalOffset = loopRadius * std::sin(angle);
      gp_scalar verticalOffset = -loopRadius * std::cos(angle);

      gp_vec3 point = center + headingNorm * horizontalOffset + gp_vec3(static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), verticalOffset);

      if (segment_count > 0) {
        gp_scalar segmentDist = (point - segments[segment_count-1].start).norm();
        totalDistance += segmentDist;
      }

      gp_scalar simTimeMsec = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
      segments[segment_count++] = Path(point, gp_vec3::UnitX(), totalDistance, static_cast<gp_scalar>(0.0f), simTimeMsec);
    }
  }

public:
  EmbeddedPathSelector() : segment_count(0), was_truncated(false) {}

  // Generate selected path at canonical origin (0,0,0)
  // Paths generate in canonical coordinate frame; craft at virtual (0,0,0) when armed
  // WARNING: If path exceeds MAX_EMBEDDED_PATH_SEGMENTS, it will be truncated
  void generatePath(int pathIndex, gp_scalar base = 0.0f, unsigned int seed = EMBEDDED_PATH_SEED) {
    segment_count = 0;
    was_truncated = false;
    gp_scalar totalDistance = static_cast<gp_scalar>(0.0f);

    AeroStandardPathType pathType = static_cast<AeroStandardPathType>(pathIndex % static_cast<int>(AERO_END_MARKER));
    gp_vec3 entryPoint(static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f));

    switch(pathType) {
      case StraightAndLevel: {
        // Racetrack pattern
        addStraightSegment(entryPoint, gp_vec3(static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)), static_cast<gp_scalar>(20.0f), totalDistance);

        gp_vec3 turn1Start = segments[segment_count-1].start;
        addHorizontalTurn(turn1Start, static_cast<gp_scalar>(20.0f), static_cast<gp_scalar>(M_PI), true, totalDistance);

        gp_vec3 northStart = segments[segment_count-1].start;
        addStraightSegment(northStart, gp_vec3(static_cast<gp_scalar>(1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)), static_cast<gp_scalar>(40.0f), totalDistance);

        gp_vec3 turn2Start = segments[segment_count-1].start;
        addHorizontalTurn(turn2Start, static_cast<gp_scalar>(20.0f), static_cast<gp_scalar>(M_PI), true, totalDistance);

        gp_vec3 returnStart = segments[segment_count-1].start;
        addStraightSegment(returnStart, gp_vec3(static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)), static_cast<gp_scalar>(20.0f), totalDistance);
        break;
      }

      case SpiralClimb: {
        addStraightSegment(entryPoint, gp_vec3(static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)), static_cast<gp_scalar>(20.0f), totalDistance);

        gp_vec3 spiralStart = segments[segment_count-1].start;
        gp_scalar climbAmount = static_cast<gp_scalar>(-50.0f);  // Climb 50m from z=0 to z=-50
        addSpiralTurn(spiralStart, static_cast<gp_scalar>(20.0f), static_cast<gp_scalar>(540.0f * M_PI / 180.0f), true, climbAmount, totalDistance);

        gp_vec3 northStart = segments[segment_count-1].start;
        addStraightSegment(northStart, gp_vec3(static_cast<gp_scalar>(1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)), static_cast<gp_scalar>(40.0f), totalDistance);
        break;
      }

      case HorizontalFigureEight: {
        gp_scalar loopRadius = static_cast<gp_scalar>(20.0f);

        // Lead-in
        const gp_scalar leadSeconds = static_cast<gp_scalar>(1.0f);
        const gp_scalar leadDistance = SIM_RABBIT_VELOCITY * leadSeconds;
        addStraightSegment(entryPoint, gp_vec3(static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)), leadDistance, totalDistance);

        gp_vec3 loopOrigin = segments[segment_count-1].start;

        // Left loop then right loop
        addHorizontalLoop(loopOrigin, loopRadius, false, totalDistance);
        addHorizontalLoop(loopOrigin, loopRadius, true, totalDistance);
        break;
      }

      case FortyFiveDegreeAngledLoop: {
        const gp_scalar cos45 = std::sqrt(static_cast<gp_scalar>(2.0f)) / static_cast<gp_scalar>(2.0f);
        const gp_scalar sin45 = cos45;
        gp_scalar loopRadius = static_cast<gp_scalar>(15.0f);
        gp_scalar centerAlt = -loopRadius * cos45;  // Canonical origin at z=0
        gp_scalar yOffset = loopRadius * sin45;

        for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {  // 0.05 rad - baseline from horizontal 8
          if (segment_count >= MAX_EMBEDDED_PATH_SEGMENTS) break;

          gp_scalar angle = turn - static_cast<gp_scalar>(M_PI / 2.0);
          gp_scalar x_plane = -loopRadius * std::cos(angle);
          gp_scalar z_plane = loopRadius * std::sin(angle);

          gp_vec3 interpolatedPoint(x_plane, z_plane * sin45 + yOffset, centerAlt - z_plane * cos45);

          if (segment_count > 0) {
            gp_scalar distance = (interpolatedPoint - segments[segment_count-1].start).norm();
            totalDistance += distance;
          }
          gp_scalar simTimeMsecLocal = (totalDistance / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
          segments[segment_count++] = Path(interpolatedPoint, gp_vec3::UnitX(), totalDistance, static_cast<gp_scalar>(0.0f), simTimeMsecLocal);
        }
        break;
      }

      case HighPerchSplitS: {
        // Complex multi-segment path
        addStraightSegment(entryPoint, gp_vec3(static_cast<gp_scalar>(-1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f)), static_cast<gp_scalar>(20.0f), totalDistance);

        gp_vec3 seg2Start = segments[segment_count-1].start;
        gp_scalar climbAmount = static_cast<gp_scalar>(-20.0f);
        addSpiralTurn(seg2Start, static_cast<gp_scalar>(20.0f), static_cast<gp_scalar>(M_PI), true, climbAmount, totalDistance);

        gp_vec3 seg3Start = segments[segment_count-1].start;
        gp_vec3 headingNorth(static_cast<gp_scalar>(1.0f), static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f));
        gp_scalar horizontalDist = static_cast<gp_scalar>(40.0f);
        gp_scalar verticalClimb = static_cast<gp_scalar>(-20.0f);
        gp_vec3 climbVector = headingNorth * horizontalDist + gp_vec3(static_cast<gp_scalar>(0.0f), static_cast<gp_scalar>(0.0f), verticalClimb);
        gp_scalar climbDistance = climbVector.norm();
        addStraightSegment(seg3Start, climbVector.normalized(), climbDistance, totalDistance);

        gp_vec3 seg4Start = segments[segment_count-1].start;
        addHorizontalTurn(seg4Start, static_cast<gp_scalar>(5.0f), static_cast<gp_scalar>(150.0f * M_PI / 180.0f), true, totalDistance);

        gp_scalar headingAngle = static_cast<gp_scalar>(150.0f * M_PI / 180.0f);
        gp_vec3 headingSW(std::cos(headingAngle), std::sin(headingAngle), static_cast<gp_scalar>(0.0f));

        gp_vec3 seg5Start = segments[segment_count-1].start;
        addStraightSegment(seg5Start, headingSW, static_cast<gp_scalar>(30.0f), totalDistance);

        gp_vec3 seg6Start = segments[segment_count-1].start;
        if (std::abs(headingSW[0]) > static_cast<gp_scalar>(0.001f)) {
          gp_scalar distToX20 = (static_cast<gp_scalar>(-20.0f) - seg6Start[0]) / headingSW[0];
          if (distToX20 > 0) {
            addStraightSegment(seg6Start, headingSW, distToX20, totalDistance);
          }
        }

        gp_vec3 seg7Start = segments[segment_count-1].start;
        addPitchDownLoop(seg7Start, headingSW, static_cast<gp_scalar>(15.0f), totalDistance);

        gp_vec3 seg8Start = segments[segment_count-1].start;
        gp_vec3 headingNE = -headingSW;
        if (std::abs(headingNE[0]) > static_cast<gp_scalar>(0.001f)) {
          gp_scalar distToX40 = (static_cast<gp_scalar>(40.0f) - seg8Start[0]) / headingNE[0];
          addStraightSegment(seg8Start, headingNE.normalized(), std::abs(distToX40), totalDistance);
        }
        break;
      }

      case SeededRandomB: {
        // Generate seeded random path using desktop-compatible mt19937
        std::mt19937 rng(seed);

        gp_vec3 controlPoints[NUM_SEGMENTS_PER_PATH];
        for (int i = 0; i < NUM_SEGMENTS_PER_PATH; ++i) {
          controlPoints[i] = localRandomPointInCylinder(rng, static_cast<gp_scalar>(40.0f), static_cast<gp_scalar>(100.0f), static_cast<gp_scalar>(0.0f));
        }

        // Generate smooth path through control points
        gp_scalar odometer = static_cast<gp_scalar>(0);
        gp_scalar turnmeter = static_cast<gp_scalar>(0);
        gp_vec3 lastPoint;
        gp_vec3 lastDirection;
        bool first = true;

        for (int i = 1; i < NUM_SEGMENTS_PER_PATH - 3; ++i) {
          for (gp_scalar t = 0; t <= static_cast<gp_scalar>(1.0); t += static_cast<gp_scalar>(0.05f)) {
            if (segment_count >= MAX_EMBEDDED_PATH_SEGMENTS) goto exitLoop;

            gp_vec3 interpolatedPoint = cubicInterpolate(controlPoints[i - 1], controlPoints[i],
                                                         controlPoints[i + 1], controlPoints[i + 2], t);
            if (!first) {
              gp_scalar newDistance = (interpolatedPoint - lastPoint).norm();
              gp_vec3 newDirection = (interpolatedPoint - lastPoint).normalized();

              gp_scalar simTimeMsec = (odometer / SIM_RABBIT_VELOCITY) * static_cast<gp_scalar>(1000.0f);
              segments[segment_count++] = Path(interpolatedPoint, gp_vec3::UnitX(), odometer, turnmeter, simTimeMsec);

              odometer += newDistance;
              lastDirection = newDirection;
            } else {
              first = false;
              lastDirection = (interpolatedPoint - entryPoint).normalized();
            }
            lastPoint = interpolatedPoint;
          }
        }
      exitLoop:
        break;
      }
    }
  }

  int getSegmentCount() const { return segment_count; }

  const Path& getSegment(int index) const {
    if (index < 0) index = 0;
    if (index >= segment_count) index = segment_count - 1;
    return segments[index];
  }

  // Copy segments to vector (for compatibility with existing code)
  void copyToVector(std::vector<Path>& path_vector) const {
    path_vector.clear();
    for (int i = 0; i < segment_count; i++) {
      path_vector.push_back(segments[i]);
    }
  }

  gp_scalar getTotalPathLength() const {
    return segment_count > 0 ? segments[segment_count - 1].distanceFromStart : static_cast<gp_scalar>(0.0f);
  }

  bool wasTruncated() const { return was_truncated; }
};

#endif // EMBEDDED_PATHGEN_SELECTOR_H
