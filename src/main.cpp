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
  mspUpdateState();
  controllerUpdate();
  mspSetControls();
}
