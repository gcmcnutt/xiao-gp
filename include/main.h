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

// interfaces
void msplinkSetup();
void msplinkLoop();
void ledSetup();
void heartBeatLED();
void blueToothSetup();
void blueToothLoop();