#include <main.h>

void setup()
{
  // first thing to do, show a RED ledfrom off to red
  ledSetup();

  // logger
  consoleInit();

  // msp link
  msplinkSetup();

  // bluetooth sync handler
  blueToothSetup();

  // flight controller
  controllerSetup();
}

void loop()
{
  heartBeatLED();
  blueToothLoop();
  
  // change these to be a single timer based loop
  // collect all the data
  // then update the state
  // then update the controller
  // then send the controls

  mspUpdateState();
  controllerUpdate();
  mspSetControls();
}
