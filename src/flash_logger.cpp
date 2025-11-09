#include "main.h"
#include <nrfx_qspi.h>
#include <string.h>

// Flash layout
// Block 0: Metadata (flight counter, file index)
// Blocks 1+: Log data
#define FLASH_BLOCK_SIZE 4096
#define FLASH_METADATA_ADDR 0x00000000
#define FLASH_DATA_START_ADDR 0x00001000  // Start after metadata block
#define FLASH_TOTAL_SIZE (2 * 1024 * 1024)  // 2MB
#define MAX_FLIGHT_INDEX 100  // Track up to 100 flights

// Flight index entry
struct FlightEntry {
  uint32_t startAddr;
  uint32_t endAddr;
  uint32_t flightNumber;
};

// Metadata structure
struct FlashMetadata {
  uint32_t magic;           // 0xF117DA7A
  uint32_t flightCounter;   // Current flight number
  uint32_t currentWriteAddr; // Next write address
  uint32_t currentFileStartAddr; // Start of current log file
  uint32_t numFlights;      // Number of flights in index
  FlightEntry flightIndex[MAX_FLIGHT_INDEX];
  char padding[FLASH_BLOCK_SIZE - 20 - (MAX_FLIGHT_INDEX * sizeof(FlightEntry))];
};

// Global state
static char writeBuffer[FLASH_LOGGER_BUFFER_SIZE];
static uint32_t bufferPos = 0;
static FlashMetadata metadata;
bool flashInitialized = false;  // Non-static for debug access
bool flashFull = false;          // Non-static for debug access
static FlashLoggerState currentState = FLASH_IDLE;
uint32_t writeCallCount = 0;    // Non-static for debug access

// Download state
static uint32_t downloadReadAddr = 0;
static uint32_t downloadEndAddr = 0;

// Forward declarations
static bool qspiInit();
static bool qspiRead(uint32_t addr, void* data, size_t len);
static bool qspiWrite(uint32_t addr, const void* data, size_t len);
static bool qspiErase(uint32_t addr);
static bool loadMetadata();
static bool saveMetadata();
static void flushBuffer();

// QSPI Configuration
static nrfx_qspi_config_t qspiConfig = {
  .pins = {
    .sck_pin = 21,
    .csn_pin = 25,
    .io0_pin = 20,
    .io1_pin = 24,
    .io2_pin = 22,
    .io3_pin = 23,
  },
  .prot_if = {
    .readoc = NRF_QSPI_READOC_READ4O,
    .writeoc = NRF_QSPI_WRITEOC_PP4O,
    .addrmode = NRF_QSPI_ADDRMODE_24BIT,
    .dpmconfig = false,
  },
  .phy_if = {
    .sck_delay = 0,
    .dpmen = false,
    .spi_mode = NRF_QSPI_MODE_0,
    .sck_freq = NRF_QSPI_FREQ_32MDIV1
  },
  .irq_priority = 6
};

void flashLoggerInit() {
  if (flashInitialized) {
    Serial.println("DEBUG: flashLoggerInit() - already initialized");
    return;
  }

  Serial.println("DEBUG: flashLoggerInit() - starting init");

  // Initialize QSPI
  if (!qspiInit()) {
    logPrint(ERROR, "QSPI init failed");
    Serial.println("DEBUG: QSPI init FAILED");
    return;
  }
  Serial.println("DEBUG: QSPI init success");

  // Load metadata
  if (!loadMetadata()) {
    // First boot or corrupted metadata
    Serial.print("DEBUG: loadMetadata() FAILED - magic=0x");
    Serial.println(metadata.magic, HEX);
    logPrint(WARNING, "Flash not initialized! Use 'Erase All Logs' via BLE to initialize.");
    logPrint(WARNING, "Logging is DISABLED until flash is erased/initialized.");

    // Set to safe defaults but don't enable logging
    metadata.magic = 0x00000000; // Invalid magic = not initialized
    metadata.flightCounter = 0;
    metadata.currentWriteAddr = FLASH_DATA_START_ADDR;
    metadata.currentFileStartAddr = FLASH_DATA_START_ADDR;
    flashInitialized = false;
    flashFull = true; // Prevent writes until properly initialized
    Serial.println("DEBUG: flashInitialized=false, flashFull=true");
    return;
  }

  Serial.print("DEBUG: loadMetadata() SUCCESS - magic=0x");
  Serial.print(metadata.magic, HEX);
  Serial.print(" flight=");
  Serial.println(metadata.flightCounter);

  // Close out the previous flight if it exists
  if (metadata.flightCounter > 0 && metadata.currentWriteAddr > metadata.currentFileStartAddr) {
    // Previous flight has data, add to index
    if (metadata.numFlights < MAX_FLIGHT_INDEX) {
      metadata.flightIndex[metadata.numFlights].flightNumber = metadata.flightCounter;
      metadata.flightIndex[metadata.numFlights].startAddr = metadata.currentFileStartAddr;
      metadata.flightIndex[metadata.numFlights].endAddr = metadata.currentWriteAddr;
      metadata.numFlights++;
      logPrint(INFO, "Archived flight #%lu to index (%lu flights total)",
               (unsigned long)metadata.flightCounter, (unsigned long)metadata.numFlights);
    } else {
      logPrint(WARNING, "Flight index full, oldest flight will be lost");
    }
  }

  // Increment flight counter for new log file
  metadata.flightCounter++;
  metadata.currentFileStartAddr = metadata.currentWriteAddr;
  saveMetadata();

  flashInitialized = true;
  bufferPos = 0;
  flashFull = false;

  Serial.println("DEBUG: flashInitialized=true, flashFull=false");
  logPrint(INFO, "Flash logger initialized, flight #%lu", (unsigned long)metadata.flightCounter);
}

void flashLoggerWrite(const char* msg) {
  writeCallCount++;

  if (writeCallCount % 100 == 1) {  // Every 100th call
    Serial.print("DEBUG: flashLoggerWrite call #");
    Serial.print(writeCallCount);
    Serial.print(" init=");
    Serial.print(flashInitialized);
    Serial.print(" full=");
    Serial.println(flashFull);
  }

  if (!flashInitialized || flashFull) {
    return;
  }

  size_t msgLen = strlen(msg);
  if (msgLen == 0) {
    return;
  }

  // Add newline if not present
  bool needsNewline = (msg[msgLen - 1] != '\n');
  size_t totalLen = msgLen + (needsNewline ? 1 : 0);

  // Check if buffer would overflow
  if (bufferPos + totalLen >= FLASH_LOGGER_BUFFER_SIZE) {
    // Flush current buffer first
    flushBuffer();
  }

  // Check if still won't fit (message too large)
  if (totalLen >= FLASH_LOGGER_BUFFER_SIZE) {
    logPrint(WARNING, "Log message too large, skipping");
    return;
  }

  // Append to buffer
  memcpy(writeBuffer + bufferPos, msg, msgLen);
  bufferPos += msgLen;

  if (needsNewline) {
    writeBuffer[bufferPos++] = '\n';
  }
}

void flashLoggerFlushCheck() {
  if (!flashInitialized || flashFull || bufferPos == 0) {
    return;
  }

  // Only flush if:
  // 1. Buffer is getting full (>3KB = 75% full) - must flush to prevent overflow
  // 2. OR buffer has some data and we have spare time (loop running fast)
  static unsigned long lastFlushTime = 0;
  unsigned long now = millis();
  unsigned long timeSinceLastFlush = now - lastFlushTime;

  bool bufferAlmostFull = (bufferPos >= 3072);  // 3KB threshold
  bool haveSpareTime = (timeSinceLastFlush >= 100);  // Flush at most every 100ms

  if (bufferAlmostFull || haveSpareTime) {
    Serial.print("DEBUG: Flushing (bufferPos=");
    Serial.print(bufferPos);
    Serial.print(" timeSince=");
    Serial.print(timeSinceLastFlush);
    Serial.println("ms)");

    unsigned long flushStartTime = micros();
    flushBuffer();
    unsigned long flushDuration = micros() - flushStartTime;

    Serial.print("DEBUG: Flush took ");
    Serial.print(flushDuration);
    Serial.println(" us");

    lastFlushTime = now;
  }
}

void flashLoggerErase() {
  // Allow erase even if not initialized (this initializes the flash)
  currentState = FLASH_ERASING;
  logPrint(INFO, "Erasing flash logs...");

  Serial.println("DEBUG: Erasing data blocks from 0x1000 to 0x200000");
  unsigned long startTime = millis();
  int blockCount = 0;

  // Erase all data blocks
  for (uint32_t addr = FLASH_DATA_START_ADDR; addr < FLASH_TOTAL_SIZE; addr += 64 * 1024) {
    qspiErase(addr);
    blockCount++;
    if (blockCount % 8 == 0) {
      Serial.print(".");  // Progress indicator every 512KB
    }
  }

  unsigned long elapsedMs = millis() - startTime;
  Serial.println();
  Serial.print("DEBUG: Data blocks erased in ");
  Serial.print(elapsedMs);
  Serial.print("ms (");
  Serial.print(blockCount);
  Serial.println(" blocks)");

  // Initialize metadata with proper magic number
  metadata.magic = 0xF117DA7A;
  metadata.flightCounter = 0;
  metadata.currentWriteAddr = FLASH_DATA_START_ADDR;
  metadata.currentFileStartAddr = FLASH_DATA_START_ADDR;
  metadata.numFlights = 0;

  // Clear the flight index
  memset(metadata.flightIndex, 0, sizeof(metadata.flightIndex));

  Serial.print("DEBUG: About to save metadata with magic=0x");
  Serial.println(metadata.magic, HEX);
  bool saved = saveMetadata();
  Serial.print("DEBUG: saveMetadata() returned: ");
  Serial.println(saved ? "SUCCESS" : "FAILED");

  logPrint(INFO, "Flash erased and initialized");
  // Note: Reboot is now handled by caller (bluetooth.cpp) to avoid blocking delay
}

bool flashLoggerIsFull() {
  return flashFull;
}

uint32_t flashLoggerGetCurrentFlightNumber() {
  return metadata.flightCounter;
}

int flashLoggerGetFileCount() {
  // Return archived flights + current flight (if it has data)
  int count = metadata.numFlights;
  if (metadata.flightCounter > 0 &&
      metadata.currentWriteAddr > metadata.currentFileStartAddr) {
    count++; // Current flight has data
  }
  return count;
}

const char* flashLoggerGetFileName(int index) {
  static char filename[32];

  // Check if this is an archived flight
  if (index < (int)metadata.numFlights) {
    snprintf(filename, sizeof(filename), "flight_%03lu.txt",
             (unsigned long)metadata.flightIndex[index].flightNumber);
    return filename;
  }

  // Check if this is the current flight
  if (index == (int)metadata.numFlights && metadata.flightCounter > 0 &&
      metadata.currentWriteAddr > metadata.currentFileStartAddr) {
    snprintf(filename, sizeof(filename), "flight_%03lu.txt",
             (unsigned long)metadata.flightCounter);
    return filename;
  }

  return nullptr;
}

uint32_t flashLoggerGetFileSize(int index) {
  // Flush any pending data first so we get accurate size
  if (bufferPos > 0) {
    flushBuffer();
  }

  // Archived flight
  if (index < (int)metadata.numFlights) {
    return metadata.flightIndex[index].endAddr - metadata.flightIndex[index].startAddr;
  }

  // Current flight
  if (index == (int)metadata.numFlights) {
    return metadata.currentWriteAddr - metadata.currentFileStartAddr;
  }

  return 0;
}

bool flashLoggerStartDownload(const char* filename) {
  if (!flashInitialized || currentState != FLASH_IDLE) {
    return false;
  }

  // Flush any pending data before download
  if (bufferPos > 0) {
    flushBuffer();
    saveMetadata(); // Ensure metadata reflects flushed data
  }

  // Parse flight number from filename (e.g., "flight_001.txt")
  uint32_t requestedFlight = 0;
  if (sscanf(filename, "flight_%lu.txt", (unsigned long*)&requestedFlight) != 1) {
    logPrint(ERROR, "Invalid filename format: %s", filename);
    return false;
  }

  // Search for this flight in the index
  for (uint32_t i = 0; i < metadata.numFlights; i++) {
    if (metadata.flightIndex[i].flightNumber == requestedFlight) {
      downloadReadAddr = metadata.flightIndex[i].startAddr;
      downloadEndAddr = metadata.flightIndex[i].endAddr;
      currentState = FLASH_DOWNLOADING;
      logPrint(INFO, "Starting download: %s (%lu bytes)",
               filename, (unsigned long)(downloadEndAddr - downloadReadAddr));
      return true;
    }
  }

  // Check if it's the current flight
  if (requestedFlight == metadata.flightCounter) {
    downloadReadAddr = metadata.currentFileStartAddr;
    downloadEndAddr = metadata.currentWriteAddr;

    // Validate addresses
    if (downloadReadAddr >= downloadEndAddr) {
      logPrint(WARNING, "Flight has no data (start=%lu, end=%lu)",
               (unsigned long)downloadReadAddr, (unsigned long)downloadEndAddr);
      return false;
    }

    currentState = FLASH_DOWNLOADING;
    logPrint(INFO, "Starting download: %s (%lu bytes) addr 0x%lx-0x%lx",
             filename, (unsigned long)(downloadEndAddr - downloadReadAddr),
             (unsigned long)downloadReadAddr, (unsigned long)downloadEndAddr);
    return true;
  }

  logPrint(ERROR, "Flight #%lu not found", (unsigned long)requestedFlight);
  return false;
}

int flashLoggerReadChunk(uint8_t* buffer, size_t maxLen) {
  if (currentState != FLASH_DOWNLOADING) {
    return -1;
  }

  if (downloadReadAddr >= downloadEndAddr) {
    // Download complete
    uint32_t totalSent = downloadReadAddr - downloadEndAddr + (downloadEndAddr - downloadReadAddr);
    logPrint(DEBUG, "Download complete, total bytes: %lu", (unsigned long)totalSent);
    return 0;
  }

  // Read chunk
  size_t toRead = downloadEndAddr - downloadReadAddr;
  if (toRead > maxLen) {
    toRead = maxLen;
  }

  if (!qspiRead(downloadReadAddr, buffer, toRead)) {
    logPrint(ERROR, "QSPI read failed at addr 0x%lx", (unsigned long)downloadReadAddr);
    return -1;
  }

  downloadReadAddr += toRead;

  // Trim trailing null padding from last chunk ONLY
  if (downloadReadAddr >= downloadEndAddr && toRead > 0) {
    size_t originalLen = toRead;
    // Remove trailing nulls
    while (toRead > 0 && buffer[toRead - 1] == 0) {
      toRead--;
    }
    if (originalLen != toRead) {
      logPrint(DEBUG, "Trimmed %lu trailing nulls from last chunk",
               (unsigned long)(originalLen - toRead));
    }
  }

  return toRead;
}

void flashLoggerStopDownload() {
  currentState = FLASH_IDLE;
  downloadReadAddr = 0;
  downloadEndAddr = 0;
}

FlashLoggerState flashLoggerGetState() {
  return currentState;
}

// Internal functions

static bool qspiInit() {
  nrfx_err_t err = nrfx_qspi_init(&qspiConfig, nullptr, nullptr);
  if (err != NRFX_SUCCESS) {
    return false;
  }
  return true;
}

static bool qspiRead(uint32_t addr, void* data, size_t len) {
  nrfx_err_t err = nrfx_qspi_read(data, len, addr);
  if (err != NRFX_SUCCESS) {
    return false;
  }

  // Wait for read to complete (reads are also async!)
  while (nrfx_qspi_mem_busy_check()) {
    delayMicroseconds(10);
  }

  return true;
}

static bool qspiWrite(uint32_t addr, const void* data, size_t len) {
  nrfx_err_t err = nrfx_qspi_write(data, len, addr);
  if (err != NRFX_SUCCESS) {
    return false;
  }

  // Wait for write to complete (writes are also async)
  while (nrfx_qspi_mem_busy_check()) {
    delayMicroseconds(10);
  }

  return true;
}

static bool qspiErase(uint32_t addr) {
  // Erase 64KB sector
  nrf_qspi_erase_len_t eraseLen = NRF_QSPI_ERASE_LEN_64KB;
  nrfx_err_t err = nrfx_qspi_erase(eraseLen, addr);
  if (err != NRFX_SUCCESS) {
    return false;
  }

  // CRITICAL: nrfx_qspi_erase is async - must wait for completion!
  // Poll status register until erase finishes (busy_check returns true while busy)
  while (nrfx_qspi_mem_busy_check()) {
    // Wait for erase to complete
    delayMicroseconds(100);
  }

  return true;
}

static bool loadMetadata() {
  if (!qspiRead(FLASH_METADATA_ADDR, &metadata, sizeof(metadata))) {
    return false;
  }

  // Check magic number
  if (metadata.magic != 0xF117DA7A) {
    return false;
  }

  // Validate addresses
  if (metadata.currentWriteAddr < FLASH_DATA_START_ADDR ||
      metadata.currentWriteAddr >= FLASH_TOTAL_SIZE) {
    return false;
  }

  return true;
}

static bool saveMetadata() {
  // Erase metadata block first
  if (!qspiErase(FLASH_METADATA_ADDR)) {
    return false;
  }

  return qspiWrite(FLASH_METADATA_ADDR, &metadata, sizeof(metadata));
}

static void flushBuffer() {
  if (bufferPos == 0) {
    return;
  }

  Serial.print("DEBUG: flushBuffer() bufferPos=");
  Serial.print(bufferPos);
  Serial.print(" writeAddr=0x");
  Serial.println(metadata.currentWriteAddr, HEX);

  // Align to 4-byte boundary (QSPI requirement)
  size_t alignedLen = (bufferPos + 3) & ~3;

  // Pad with zeros
  for (size_t i = bufferPos; i < alignedLen; i++) {
    writeBuffer[i] = 0;
  }

  Serial.print("DEBUG: Writing ");
  Serial.print(alignedLen);
  Serial.print(" bytes (");
  Serial.print(bufferPos);
  Serial.println(" actual + padding)");

  // Check if we have space
  if (metadata.currentWriteAddr + alignedLen >= FLASH_TOTAL_SIZE) {
    logPrint(WARNING, "Flash full, stopping logging");
    flashFull = true;
    bufferPos = 0;
    return;
  }

  // Write to flash
  bool writeSuccess = qspiWrite(metadata.currentWriteAddr, writeBuffer, alignedLen);
  Serial.print("DEBUG: qspiWrite returned: ");
  Serial.println(writeSuccess ? "SUCCESS" : "FAILED");

  if (!writeSuccess) {
    logPrint(ERROR, "Flash write failed");
    // Retry once
    if (!qspiWrite(metadata.currentWriteAddr, writeBuffer, alignedLen)) {
      logPrint(ERROR, "Flash write retry failed, discarding buffer");
      bufferPos = 0;
      return;
    }
  }

  // Update write position
  metadata.currentWriteAddr += alignedLen;

  // Save metadata more frequently (every 4KB) to keep it in sync
  static uint32_t lastMetadataSave = 0;
  if (metadata.currentWriteAddr - lastMetadataSave >= 4096) {
    saveMetadata();
    lastMetadataSave = metadata.currentWriteAddr;
  }

  bufferPos = 0;
}
