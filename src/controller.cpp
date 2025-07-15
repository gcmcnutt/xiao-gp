#include <main.h>
#include <ArduinoEigenDense.h>

#include "GP/autoc/aircraft_state.h"
#include "gp_evaluator.h"

using namespace Eigen;

AircraftState aircraftState;
std::vector<Path> flightPath;

void controllerSetup()
{
  // Initialize flight path (example: circular path)
  // In practice, this would be loaded from flash memory or generated based on mission parameters
  flightPath.clear();
  
  for (int i = 0; i < 20; i++) {
    Path waypoint;
    double angle = (i * 2.0 * M_PI) / 20.0;
    waypoint.start = Vector3d(
      30.0 * cos(angle),  // circular path with 30m radius
      30.0 * sin(angle),
      -10.0  // constant altitude
    );
    waypoint.orientation = Vector3d(cos(angle + M_PI/2), sin(angle + M_PI/2), 0);
    waypoint.distanceFromStart = i * (2.0 * M_PI * 30.0) / 20.0;  // circumference / segments
    waypoint.radiansFromStart = angle;
    
    flightPath.push_back(waypoint);
  }
  
  logPrint(INFO, "GP Controller initialized with %d waypoints", flightPath.size());
}

void controllerUpdate()
{
  static unsigned long lastQueryTime = 0;
  unsigned long now = millis();
  if (now - lastQueryTime >= MSP_UPDATE_INTERVAL_MSEC)
  {
    lastQueryTime = now;

    mspUpdateState();

    // Convert msp state to aircraft state
    aircraftState.setOrientation(
        Quaterniond(state.attitude_quaternion.q[0], state.attitude_quaternion.q[1], state.attitude_quaternion.q[2], state.attitude_quaternion.q[3]));

    // Given a lattiture and longitude, convert to a x, y position measured in meters relative to a zero reference
    float lat = state.raw_gps.lat;
    float lon = state.raw_gps.lon;
    float alt = state.raw_gps.alt;

    // TODO get these out of the MSP data
    float homeX = 0;
    float homeY = 0;
    float homeZ = 0;

    // for now use flat earth approximation
    // TODO use haversine formula to calculate distance between two points
    float R = (6371000 * M_PI / 180); // Earth radius in meters to radians
    float y = R * (lat - homeX);
    float x = R * (lon - homeY) * cos(homeX); // TODO this should be lat_avg = (lat_home + lat_curr) / 2 (average latitude in degrees, converted to radians for cos).
    float z = alt - homeZ;

    aircraftState.setPosition(Vector3d(x, y, z));

    aircraftState.setSimTimeMsec(state.asOfMsec);
    // TODO virtual pitot: aircraftState.setRelVel(...);
    
    // Find closest waypoint for path index
    updatePathIndex();
    
    // Run the compiled GP evaluator to compute control commands
    double gpResult = evaluateGP(aircraftState, flightPath, 0.0);
    
    // The GP sets control commands via aircraftState.set*Command() calls
    // Extract the commands for MSP output
    logPrint(DEBUG, "GP result: %.3f, Commands - P:%.3f R:%.3f T:%.3f", 
             gpResult, 
             aircraftState.getPitchCommand(), 
             aircraftState.getRollCommand(), 
             aircraftState.getThrottleCommand());
  }
}

// Helper function to update the current path index based on position
void updatePathIndex() {
  if (flightPath.empty()) return;
  
  double minDistance = 1e6;
  int closestIndex = 0;
  
  for (int i = 0; i < flightPath.size(); i++) {
    double distance = (flightPath[i].start - aircraftState.getPosition()).norm();
    if (distance < minDistance) {
      minDistance = distance;
      closestIndex = i;
    }
  }
  
  aircraftState.setThisPathIndex(closestIndex);
}