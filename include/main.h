#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <HardwareSerial.h>
#include "MSP.h"
#include "state.h"
#include "flash_logger.h"
#include <GP/autoc/gp_types.h>

// Forward declarations
struct AircraftState;

// Define RGB pins
#define RED_PIN 11
#define GREEN_PIN 12
#define BLUE_PIN 13

#define HEARTBEAT_LED RED_PIN
#define BLINK_INTERVAL_MSEC 250

#define MSP_UPDATE_INTERVAL_MSEC 200
#define MSP_SEND_INTERVAL_MSEC 50
#define MSP_REPLY_TIMEOUT_MSEC 50
#define MSP_LOS_INTERVAL_MSEC 2000

#define MSP_DEFAULT_CHANNEL_VALUE 1500
#define MSP_ARM_CHANNEL 8
#define MSP_ARMED_THRESHOLD 1600
#define MSP_ARM_CYCLE_COUNT 2

// MSP flight mode flags
#define MSP_MODE_MSPRCOVERRIDE 30

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
void blueToothSetEnabled(bool enabled);

// GP rabbit path following functions
void convertMSPStateToAircraftState(AircraftState& aircraftState);
int getRabbitPathIndex(unsigned long elapsed_msec);
int convertRollToMSPChannel(gp_scalar gp_command);
int convertPitchToMSPChannel(gp_scalar gp_command);
int convertThrottleToMSPChannel(gp_scalar gp_command);
