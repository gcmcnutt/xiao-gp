#pragma once

#include <Arduino.h>

// Flash logger configuration
#define FLASH_LOGGER_BUFFER_SIZE 4096  // 4KB RAM buffer
#define FLASH_LOGGER_CHUNK_SIZE 128    // BLE transfer chunk size (smaller for reliability)
#define FLASH_MAX_FILES 999            // flight_001.txt to flight_999.txt

// Flash logger states
enum FlashLoggerState {
  FLASH_IDLE,
  FLASH_DOWNLOADING,
  FLASH_ERASING
};

// Flash logger API
void flashLoggerInit();
uint32_t flashLoggerWrite(const char* msg);
void flashLoggerFlushCheck();
void flashLoggerErase();
bool flashLoggerIsFull();
bool flashLoggerIsSuspended();

// File transfer API (for BLE)
int flashLoggerGetFileCount();
const char* flashLoggerGetFileName(int index);
uint32_t flashLoggerGetFileSize(int index);
uint32_t flashLoggerGetCurrentFlightNumber();
bool flashLoggerStartDownload(const char* filename);
int flashLoggerReadChunk(uint8_t* buffer, size_t maxLen);
void flashLoggerStopDownload();
FlashLoggerState flashLoggerGetState();
bool flashLoggerHasPendingData();
uint32_t flashLoggerGetActiveDownloadSize();
