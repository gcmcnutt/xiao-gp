#include <main.h>

void setup()
{
  // first thing to do, show a RED ledfrom off to red
  ledSetup();

  // logger
  consoleInit();

  // Wait for WSL serial port to reconnect after reboot
  delay(8000);
  Serial.println("\n\n========== XIAO-GP BOOT ==========");

  // flash logger (init after console for debug output)
  flashLoggerInit();

  // msp link
  msplinkSetup();

  // bluetooth sync handler
  blueToothSetup();

  // flight controller
  controllerSetup();

  Serial.println("========== BOOT COMPLETE ==========\n");
}

void loop()
{
  heartBeatLED();
  blueToothLoop();

  controllerUpdate();

  // Flush flash logger at end of loop
  flashLoggerFlushCheck();
}
