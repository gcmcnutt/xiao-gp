#include <main.h>
#include <stdarg.h>

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

void consoleInit() {
  Serial.begin(115200);
  // while (!Serial)
  // {
  //   ; // Wait for serial connection (optional)
  // }
}

// Default log level (can be changed at runtime)
static LogLevel currentLogLevel = INFO;

// Buffer for formatted output
char logBuffer[512];

// Logger function with variable arguments
void logPrint(LogLevel level, const char* format, ...) {
  // Only print if level is high enough
  if (level < currentLogLevel) {
      return;
  }

  // Get current time in milliseconds
  unsigned long timestamp = millis();

  // Create prefix with timestamp, INAV sample time, and level
  char prefix[48];
  const char* levelStr;

  switch(level) {
      case DEBUG:   levelStr = "d";   break;
      case INFO:    levelStr = "i";    break;
      case WARNING: levelStr = "w"; break;
      case ERROR:   levelStr = "e";   break;
      default:      levelStr = "?"; break;
  }

  snprintf(prefix, sizeof(prefix), "%07lu %07lu %s ", timestamp, state.inavSampleTimeMsec, levelStr);

  // Handle the variable arguments
  va_list args;
  va_start(args, format);
  
  // First print to buffer
  vsnprintf(logBuffer, sizeof(logBuffer), format, args);

  // Combine prefix and message for flash/Serial output
  char flashBuffer[560]; // prefix(48) + logBuffer(512)
  snprintf(flashBuffer, sizeof(flashBuffer), "%s%s", prefix, logBuffer);

  // Write to flash logger and get monotonic message ID
  uint32_t messageId = flashLoggerWrite(flashBuffer);

  // Output to Serial with a shorter id prefix (6 digits, no '#')
  char idPrefix[12];
  snprintf(idPrefix, sizeof(idPrefix), "%06lu ", (unsigned long)(messageId % 1000000UL));
  Serial.print(idPrefix);
  Serial.println(flashBuffer);

  va_end(args);
}

// Optional: Function to set log level
void setLogLevel(LogLevel level) {
  currentLogLevel = level;
}
