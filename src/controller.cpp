#include <main.h>
#include <ArduinoEigenDense.h>

#include "autoc/aircraft_state.h"

using namespace Eigen;

AircraftState aircraftState;

void controllerSetup()
{
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
  }
}