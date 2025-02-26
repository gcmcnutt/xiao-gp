#include <main.h>

void ledSetup()
{
  // Initialize pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  analogWrite(GREEN_PIN, 255); // Green
  analogWrite(BLUE_PIN, 255);  // Blue
  analogWrite(RED_PIN, 0);     // Red
}

void heartBeatLED()
{
  static unsigned long lastBlinkTime = 0;
  if (millis() - lastBlinkTime >= BLINK_INTERVAL_MSEC)
  {
    lastBlinkTime = millis();
    static bool ledState = false;
    ledState = !ledState;
    if (ledState)
    {
      analogWrite(HEARTBEAT_LED, 255);
    }
    else
    {
      analogWrite(HEARTBEAT_LED, 0);
    }
  }
}
