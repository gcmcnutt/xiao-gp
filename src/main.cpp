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
#ifdef USE_MSP_SIMULATION
  // Wait 8 seconds for USB monitor to initialize in simulation mode
  static uint32_t sim_start_time = 0;
  static bool delay_message_shown = false;

  if (sim_start_time == 0) {
    sim_start_time = millis();
  }

  if (millis() - sim_start_time < 8000) {
    if (!delay_message_shown) {
      Serial.println("MSP Simulation: Waiting 8 seconds for USB monitor initialization...");
      delay_message_shown = true;
    }
    heartBeatLED(); // Keep LED heartbeat active during delay
    return; // Skip main loop until 8 seconds have elapsed
  }

  if (delay_message_shown) {
    Serial.println("MSP Simulation: Initialization delay complete, starting flight simulation");
    // Reset MSP simulation timing to account for the 8-second delay
    mspResetTiming();
    delay_message_shown = false; // Reset for potential future resets
  }
#endif

  heartBeatLED();
  // blueToothLoop();

  controllerUpdate();
}