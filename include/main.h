#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <HardwareSerial.h>
#include "mavlink_headers/common/mavlink.h"

// Define RGB pins
#define RED_PIN 11
#define GREEN_PIN 12
#define BLUE_PIN 13

#define HEARTBEAT_LED RED_PIN
#define BLINK_INTERVAL_MSEC 250

#define MAVLINK_SEND_INTERVAL_MSEC 200

// interfaces
void mavlinkSetup();
void mavlinkReceiveLoop();
void mavlinkSenderLoop();
void ledSetup();
void heartBeatLED();
void blueToothSetup();
void blueToothLoop();