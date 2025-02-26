#include <main.h>

void setup()
{
  // from off to red
  ledSetup();

  Serial.begin(115200);
  // while (!Serial)
  // {
  //   ; // Wait for serial connection (optional)
  // }

  mavlinkSetup();
  blueToothSetup();
}

void loop()
{
  heartBeatLED();
  mavlinkReceiveLoop();
  mavlinkSenderLoop();
  blueToothLoop();
}
