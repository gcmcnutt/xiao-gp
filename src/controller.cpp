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
  // Unified timing loop - 50ms base cycle with 200ms sensor/GP updates
  static unsigned long lastSendTime = 0;
  static unsigned long lastUpdateTime = 0;
  unsigned long now = millis();
  
  // 200ms cycle: Update sensors and GP control
  if (now - lastUpdateTime >= MSP_UPDATE_INTERVAL_MSEC)
  {
    lastUpdateTime = now;
    
    // Update sensor data, aircraft state, and GP control (all integrated)
    mspUpdateState();
  }
  
  // 50ms cycle: Send cached commands (always send after any update)
  if (now - lastSendTime >= MSP_SEND_INTERVAL_MSEC)
  {
    lastSendTime = now;
    mspSetControls();
  }
}