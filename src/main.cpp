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
  setLogLevel(DEBUG);

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
  // Print flash logger status every 10 seconds
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 10000) {
    extern bool flashInitialized;
    extern bool flashFull;
    extern uint32_t writeCallCount;
    Serial.print("\n[STATUS] FlashLogger: init=");
    Serial.print(flashInitialized ? "YES" : "NO");
    Serial.print(" full=");
    Serial.print(flashFull ? "YES" : "NO");
    Serial.print(" writes=");
    Serial.println(writeCallCount);
    lastStatusPrint = millis();
  }

  heartBeatLED();
  blueToothLoop();

  controllerUpdate();

  // Flush flash logger at end of loop
  flashLoggerFlushCheck();
}
