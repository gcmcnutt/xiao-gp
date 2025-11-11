#include <main.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

// Flight Logger Service
BLEService loggerService("F1706000-1234-5678-9ABC-DEF012345678");

// Control Characteristic - write commands (DL:filename, ERASE:ALL, LIST)
BLEStringCharacteristic controlCharacteristic("F1706001-1234-5678-9ABC-DEF012345678", BLEWrite, 64);

// Data Characteristic - notify file chunks
BLECharacteristic dataCharacteristic("F1706002-1234-5678-9ABC-DEF012345678", BLERead | BLENotify, FLASH_LOGGER_CHUNK_SIZE);

// Status Characteristic - read/notify status (IDLE, DOWNLOADING, etc)
BLEStringCharacteristic statusCharacteristic("F1706003-1234-5678-9ABC-DEF012345678", BLERead | BLENotify, 64);

// File transfer state
static uint8_t transferBuffer[FLASH_LOGGER_CHUNK_SIZE];
static bool transferInProgress = false;

// Reboot state for non-blocking reboot after erase
static unsigned long rebootScheduledTime = 0;
static bool rebootScheduled = false;

void blueToothSetup()
{
  // begin initialization
  if (!BLE.begin())
  {
    logPrint(ERROR, "starting Bluetooth® Low Energy module failed!");

    // TODO don't hang here
    while (1)
      ;
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("XIAO-GP");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the LED service
  ledService.addCharacteristic(switchCharacteristic);

  // add LED service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // Setup flight logger service
  loggerService.addCharacteristic(controlCharacteristic);
  loggerService.addCharacteristic(dataCharacteristic);
  loggerService.addCharacteristic(statusCharacteristic);
  BLE.addService(loggerService);

  // Initialize status
  statusCharacteristic.writeValue("IDLE");

  // start advertising
  BLE.advertise();

  logPrint(INFO, "BLE Peripheral (LED + FlightLogger)");
}

// Forward declaration
static void transferNextChunk();

// Helper: process logger control commands
static void processLoggerCommand(const char* cmd) {
  logPrint(DEBUG, "Logger cmd: %s", cmd);

  if (strncmp(cmd, "LIST", 4) == 0) {
    // List files - send file info for each flight
    int fileCount = flashLoggerGetFileCount();
    uint32_t currentFlight = flashLoggerGetCurrentFlightNumber();
    logPrint(INFO, "Listing %d flight logs", fileCount);

    // Send current flight number
    char currentInfo[32];
    snprintf(currentInfo, sizeof(currentInfo), "CURRENT:%lu", (unsigned long)currentFlight);
    statusCharacteristic.writeValue(currentInfo);
    delay(10);

    // Send each file as separate status notification
    for (int i = 0; i < fileCount; i++) {
      const char* filename = flashLoggerGetFileName(i);
      uint32_t size = flashLoggerGetFileSize(i);
      char fileInfo[64];
      snprintf(fileInfo, sizeof(fileInfo), "FILE:%s:%lu", filename, (unsigned long)size);
      statusCharacteristic.writeValue(fileInfo);
      delay(10); // Small delay between notifications
    }
    statusCharacteristic.writeValue("LIST_DONE");

  } else if (strncmp(cmd, "DL:", 3) == 0) {
    // Download file - start download and send ready status
    const char* filename = cmd + 3;

    // Parse flight number to find file size
    uint32_t flightNum = 0;
    uint32_t fileSize = 0;
    if (sscanf(filename, "flight_%lu.txt", (unsigned long*)&flightNum) == 1) {
      // Find this flight to get size
      int fileCount = flashLoggerGetFileCount();
      for (int i = 0; i < fileCount; i++) {
        const char* fname = flashLoggerGetFileName(i);
        if (fname && strcmp(fname, filename) == 0) {
          fileSize = flashLoggerGetFileSize(i);
          break;
        }
      }
    }

    logPrint(INFO, "Attempting download: %s (size: %lu bytes)", filename, (unsigned long)fileSize);

    if (fileSize == 0) {
      statusCharacteristic.writeValue("ERROR:File empty");
      logPrint(WARNING, "File is empty, nothing to download");
    } else if (flashLoggerStartDownload(filename)) {
      transferInProgress = true;
      uint32_t filteredSize = flashLoggerGetActiveDownloadSize();
      if (filteredSize == 0) {
        filteredSize = fileSize;
      }
      // Send file size and chunk size so browser knows what to expect
      char statusMsg[64];
      snprintf(statusMsg, sizeof(statusMsg), "READY:%lu:%d", (unsigned long)filteredSize, FLASH_LOGGER_CHUNK_SIZE);
      statusCharacteristic.writeValue(statusMsg);
      logPrint(INFO, "Download ready: %s (browser will request chunks)", filename);
    } else {
      statusCharacteristic.writeValue("ERROR:Cannot start download");
      logPrint(ERROR, "Failed to start download");
    }

  } else if (strncmp(cmd, "NEXT", 4) == 0) {
    // Browser requesting next chunk
    if (transferInProgress) {
      transferNextChunk();
    } else {
      statusCharacteristic.writeValue("ERROR:No download in progress");
    }

  } else if (strncmp(cmd, "ERASE:ALL", 9) == 0) {
    // Erase all logs
    statusCharacteristic.writeValue("ERASING");
    logPrint(INFO, "Erasing flash via BLE...");
    flashLoggerErase();
    statusCharacteristic.writeValue("ERASE_DONE");
    logPrint(INFO, "Erase complete, rebooting in 500ms...");
    // Schedule non-blocking reboot
    rebootScheduledTime = millis() + 500;
    rebootScheduled = true;

  } else if (strncmp(cmd, "REBOOT", 6) == 0) {
    // Reboot device to rotate log file
    statusCharacteristic.writeValue("REBOOT");
    logPrint(INFO, "Rebooting via BLE in 500ms...");
    // Schedule non-blocking reboot
    rebootScheduledTime = millis() + 500;
    rebootScheduled = true;

  } else {
    statusCharacteristic.writeValue("ERROR:Unknown command");
  }
}

// Helper: transfer next chunk (called on demand by browser)
static void transferNextChunk() {
  static uint32_t chunkCount = 0;
  static uint32_t totalBytesSent = 0;

  if (!transferInProgress) {
    return;
  }

  int bytesRead = flashLoggerReadChunk(transferBuffer, FLASH_LOGGER_CHUNK_SIZE);

  if (bytesRead < 0) {
    // Error
    logPrint(ERROR, "Flash read error during download (chunk #%lu, sent %lu bytes)",
             (unsigned long)chunkCount, (unsigned long)totalBytesSent);
    statusCharacteristic.writeValue("ERROR:Read failed");
    flashLoggerStopDownload();
    transferInProgress = false;
    chunkCount = 0;
    totalBytesSent = 0;
  } else if (bytesRead == 0) {
    // Complete
    logPrint(INFO, "Download complete: %lu chunks, %lu total bytes",
             (unsigned long)chunkCount, (unsigned long)totalBytesSent);
    statusCharacteristic.writeValue("DOWNLOAD_DONE");
    flashLoggerStopDownload();
    transferInProgress = false;
    chunkCount = 0;
    totalBytesSent = 0;
  } else {
    // Send chunk via notification
    dataCharacteristic.writeValue(transferBuffer, bytesRead);
    chunkCount++;
    totalBytesSent += bytesRead;
    if (chunkCount % 20 == 0) {
      logPrint(DEBUG, "Sent %lu chunks, %lu bytes so far",
               (unsigned long)chunkCount, (unsigned long)totalBytesSent);
    }
  }
}

// Non-blocking polling loop for BT work
void blueToothLoop()
{
  static BLEDevice lastCentral;
  static bool wasConnected = false;

  // Check for scheduled reboot (non-blocking)
  if (rebootScheduled && millis() >= rebootScheduledTime) {
    NVIC_SystemReset();
  }

  // Get current central
  BLEDevice central = BLE.central();

  // Check for new connection
  if (central && !wasConnected)
  {
    logPrint(DEBUG, "Connected to central: %s", central.address());
    lastCentral = central;
    wasConnected = true;
  }

  // If we have an active connection, service it (non-blocking)
  if (central && central.connected())
  {
    // Handle LED control
    if (switchCharacteristic.written())
    {
      if (switchCharacteristic.value())
      {
        analogWrite(BLUE_PIN, 0);
        logPrint(INFO, "LED on");
      }
      else
      {
        analogWrite(BLUE_PIN, 255);
        logPrint(INFO, "LED off");
      }
    }

    // Handle logger control commands (including NEXT for chunk requests)
    if (controlCharacteristic.written())
    {
      String cmd = controlCharacteristic.value();
      processLoggerCommand(cmd.c_str());
    }
  }
  else if (wasConnected)
  {
    // Disconnected
    logPrint(DEBUG, "Disconnected from central");
    wasConnected = false;

    // Clean up any in-progress transfer
    if (transferInProgress) {
      flashLoggerStopDownload();
      transferInProgress = false;
    }
  }
}
