#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <HardwareSerial.h>
#include "MSP.h"

// Define RGB pins
#define RED_PIN 11
#define GREEN_PIN 12
#define BLUE_PIN 13

#define HEARTBEAT_LED RED_PIN
#define BLINK_INTERVAL_MSEC 250

#define MSP_UPDATE_INTERVAL_MSEC 200
#define MSP_SEND_INTERVAL_MSEC 100
#define MSP_REPLY_TIMOUT_MSEC 50

// Define log levels
enum LogLevel
{
  DEBUG = 0,
  INFO = 1,
  WARNING = 2,
  ERROR = 3
};

// utils
void consoleInit();
void logPrint(LogLevel level, const char* format, ...);
void setLogLevel(LogLevel level);

// interfaces
void msplinkSetup();
void mspUpdateState();
void controllerSetup();
void controllerUpdate();
void mspSetControls();
void ledSetup();
void heartBeatLED();
void blueToothSetup();
void blueToothLoop();