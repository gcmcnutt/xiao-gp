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
  void generatePath(double radius = 40.0, double height = 100.0, double base = SIM_INITIAL_ALTITUDE) {
    segment_count = 0;
    double totalDistance = 0.0;
    
    // Origin point and loop parameters
    Eigen::Vector3d origin(0, 0, base);
    double loopRadius = 20.0;
    
    // Left horizontal loop (counter-clockwise)
    Eigen::Vector3d circle_center = origin + Eigen::Vector3d(0, -loopRadius, 0);
    
    for (double turn = 0; turn < 2 * M_PI && segment_count < MAX_EMBEDDED_PATH_SEGMENTS; turn += 0.05) {
      Eigen::Vector3d point = circle_center + Eigen::Vector3d(-loopRadius * sin(turn), loopRadius * cos(turn), 0);
      
      if (segment_count > 0) {
        double distance = (point - segments[segment_count-1].start).norm();
        totalDistance += distance;
      }
      
      segments[segment_count] = Path(point, Eigen::Vector3d(1, 0, 0), totalDistance, 0.0, 
                                     (totalDistance / SIM_RABBIT_VELOCITY) * 1000.0);
      segment_count++;
    }
    
    // Right horizontal loop (clockwise)
    circle_center = origin + Eigen::Vector3d(0, loopRadius, 0);
    
    for (double turn = 0; turn < 2 * M_PI && segment_count < MAX_EMBEDDED_PATH_SEGMENTS; turn += 0.05) {
      Eigen::Vector3d point = circle_center + Eigen::Vector3d(-loopRadius * sin(turn), -loopRadius * cos(turn), 0);
      
      double distance = (point - segments[segment_count-1].start).norm();
      totalDistance += distance;
      
      segments[segment_count] = Path(point, Eigen::Vector3d(1, 0, 0), totalDistance, 0.0, 
                                     (totalDistance / SIM_RABBIT_VELOCITY) * 1000.0);
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
  
  double getTotalPathLength() const {
    return segment_count > 0 ? segments[segment_count - 1].distanceFromStart : 0.0;
  }
};

#endif // EMBEDDED_PATHGEN_H