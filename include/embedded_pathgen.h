#ifndef EMBEDDED_PATHGEN_H
#define EMBEDDED_PATHGEN_H

// Embedded-friendly path generation for xiao-gp
// Uses GP constants but without heavy dependencies

#include <GP/autoc/aircraft_state.h>
#include <vector>
#include <math.h>

// Maximum number of path segments for embedded system
#define MAX_EMBEDDED_PATH_SEGMENTS 200

// Simple embedded path generator for longSequential pattern
class EmbeddedLongSequentialPath {
private:
  Path segments[MAX_EMBEDDED_PATH_SEGMENTS];
  int segment_count;
  
public:
  EmbeddedLongSequentialPath() : segment_count(0) {}
  
  // Generate longSequential path (simplified version of GP pathgen)
  void generatePath(gp_scalar radius = static_cast<gp_scalar>(40.0f),
                    gp_scalar height = static_cast<gp_scalar>(100.0f),
                    gp_scalar base = SIM_INITIAL_ALTITUDE) {
    segment_count = 0;
    gp_scalar totalDistance = 0.0f;
    
    // Origin point and loop parameters
    gp_vec3 origin(0, 0, base);
    gp_scalar loopRadius = static_cast<gp_scalar>(20.0f);
    
    // Lead-in: ~1s straight-and-level southbound to give MSP override time to take effect
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

    // Use end of lead-in as the loop origin so the horizontal 8 starts from the current location
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
  
  int getSegmentCount() const { return segment_count; }
  
  const Path& getSegment(int index) const {
    if (index < 0) index = 0;
    if (index >= segment_count) index = segment_count - 1;
    return segments[index];
  }
  
  // Copy segments to provided vector (for compatibility)
  void copyToVector(std::vector<Path>& path_vector) const {
    path_vector.clear();
    for (int i = 0; i < segment_count; i++) {
      path_vector.push_back(segments[i]);
    }
  }
  
  gp_scalar getTotalPathLength() const {
    return segment_count > 0 ? segments[segment_count - 1].distanceFromStart : static_cast<gp_scalar>(0.0f);
  }
};

#endif // EMBEDDED_PATHGEN_H
