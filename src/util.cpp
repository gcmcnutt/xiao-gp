#include <main.h>

void heartBeatLED()
{
  static unsigned long lastBlinkTime = 0;
  if (millis() - lastBlinkTime >= 250)
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
