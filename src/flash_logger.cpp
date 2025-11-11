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

static const size_t FLASH_PAGE_PROGRAM_SIZE = 256;
static const uint32_t METADATA_SAVE_INTERVAL = FLASH_BLOCK_SIZE;
static const uint32_t FLASH_SYNC_TIMEOUT_DEFAULT_MS = 250;
static const uint32_t FLASH_SYNC_POLL_DELAY_US = 50;

// Flight index entry
struct FlightEntry {
  uint32_t startAddr;
  uint32_t endAddr;
  uint32_t flightNumber;
};

// Metadata structure
struct FlashMetadata {
  uint32_t magic;            // 0xF117DA7A
  uint32_t flightCounter;    // Current flight number
  uint32_t currentWriteAddr; // Next write address (committed)
  uint32_t currentFileStartAddr; // Start of current log file
  uint32_t numFlights;       // Number of flights in index
  FlightEntry flightIndex[MAX_FLIGHT_INDEX];
  char padding[FLASH_BLOCK_SIZE - 20 - (MAX_FLIGHT_INDEX * sizeof(FlightEntry))];
};

struct BufferSlot {
  char data[FLASH_LOGGER_BUFFER_SIZE];
  size_t payloadLen;   // Actual bytes written by logger
  size_t paddedLen;    // Payload length rounded up for flash write
  size_t writeOffset;  // Bytes already written to flash
  uint32_t targetAddr; // Flash address for this buffer
  uint32_t sequence;   // Monotonic sequence for debugging
  uint32_t firstMessageId; // First message ID contained in this buffer
  uint32_t lastMessageId;  // Last message ID contained in this buffer
  uint16_t messageCount;   // Number of messages stored in this buffer
};

enum DataFlushState {
  DATA_FLUSH_IDLE,
  DATA_FLUSH_ISSUE_WRITE,
  DATA_FLUSH_WAIT_WRITE
};

enum MetadataFlushState {
  META_FLUSH_IDLE,
  META_FLUSH_ISSUE_ERASE,
  META_FLUSH_WAIT_ERASE,
  META_FLUSH_ISSUE_WRITE,
  META_FLUSH_WAIT_WRITE
};

// Global state
static BufferSlot bufferSlots[3];
static BufferSlot* activeBuffer = nullptr;
static BufferSlot* pendingBuffer = nullptr;
static BufferSlot* inflightBuffer = nullptr;
static BufferSlot* freeSlots[2];
static size_t freeSlotCount = 0;

static FlashMetadata metadata;
bool flashInitialized = false;  // Non-static for debug access
bool flashFull = false;         // Non-static for debug access
static bool flashError = false;
static FlashLoggerState currentState = FLASH_IDLE;
uint32_t writeCallCount = 0;    // Non-static for debug access

// Download state
static uint32_t downloadStartAddr = 0;
static uint32_t downloadReadAddr = 0;
static uint32_t downloadEndAddr = 0;
static uint32_t downloadFilteredSize = 0;
static uint32_t downloadBytesSent = 0;
static uint32_t downloadDebugChunksLogged = 0;

// Flush management
static DataFlushState dataFlushState = DATA_FLUSH_IDLE;
static MetadataFlushState metadataFlushState = META_FLUSH_IDLE;
static size_t inflightChunkSize = 0;
static bool flushRequested = false;
static uint32_t queuedPaddedBytes = 0;
static uint32_t nextSequenceId = 1;
static uint32_t nextMessageId = 1;
static bool metadataDirty = false;
static bool metadataPersistQueued = false;
static uint32_t metadataLastSavedAddr = FLASH_DATA_START_ADDR;

// Forward declarations
static bool qspiInit();
static bool qspiRead(uint32_t addr, void* data, size_t len);
static bool qspiWriteBlocking(uint32_t addr, const void* data, size_t len);
static bool qspiEraseBlocking(uint32_t addr, nrf_qspi_erase_len_t len);
static bool loadMetadata();
static bool saveMetadataBlocking();
static void resetBufferState();
static inline size_t alignToWord(size_t len);
static BufferSlot* acquireFreeSlot();
static void releaseSlot(BufferSlot* slot);
static bool tryStageActiveBuffer();
static void finalizeInflightBuffer();
static void serviceFlash(bool flushIntent);
static void serviceDataFlush();
static void serviceMetadataFlush();
static bool isFlashBusy();
static bool isFlushIdle();
static bool flashLoggerSyncBlocking(uint32_t timeoutMs);
static void resetSlotMetadata(BufferSlot* slot);
static bool appendMessageToActive(uint32_t msgId, const char* msg, size_t msgLen, bool needsNewline);
static void handleDroppedRange(uint32_t dropStartId, uint32_t dropEndId);
static void enqueueDropNotice(uint32_t dropStartId, uint32_t dropEndId);
static uint32_t computeFilteredSize(uint32_t startAddr, uint32_t endAddr);

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

  if (!qspiInit()) {
    logPrint(ERROR, "QSPI init failed");
    Serial.println("DEBUG: QSPI init FAILED");
    return;
  }
  Serial.println("DEBUG: QSPI init success");

  if (!loadMetadata()) {
    Serial.print("DEBUG: loadMetadata() FAILED - magic=0x");
    Serial.println(metadata.magic, HEX);
    logPrint(WARNING, "Flash not initialized! Use 'Erase All Logs' via BLE to initialize.");
    logPrint(WARNING, "Logging is DISABLED until flash is erased/initialized.");

    metadata.magic = 0x00000000;
    metadata.flightCounter = 0;
    metadata.currentWriteAddr = FLASH_DATA_START_ADDR;
    metadata.currentFileStartAddr = FLASH_DATA_START_ADDR;
    metadata.numFlights = 0;

    flashInitialized = false;
    flashFull = true;
    flashError = true;
    Serial.println("DEBUG: flashInitialized=false, flashFull=true");
    return;
  }

  Serial.print("DEBUG: loadMetadata() SUCCESS - magic=0x");
  Serial.print(metadata.magic, HEX);
  Serial.print(" flight=");
  Serial.println(metadata.flightCounter);

  flashError = false;
  flashFull = false;

  if (metadata.flightCounter > 0 && metadata.currentWriteAddr > metadata.currentFileStartAddr) {
    if (metadata.numFlights < MAX_FLIGHT_INDEX) {
      metadata.flightIndex[metadata.numFlights].flightNumber = metadata.flightCounter;
      metadata.flightIndex[metadata.numFlights].startAddr = metadata.currentFileStartAddr;
      metadata.flightIndex[metadata.numFlights].endAddr = metadata.currentWriteAddr;
      metadata.numFlights++;
      logPrint(INFO, "Archived flight #%lu to index (%lu flights total)",
               (unsigned long)metadata.flightCounter,
               (unsigned long)metadata.numFlights);
    } else {
      logPrint(WARNING, "Flight index full, oldest flight will be lost");
    }
  }

  metadata.flightCounter++;
  metadata.currentFileStartAddr = metadata.currentWriteAddr;
  if (!saveMetadataBlocking()) {
    logPrint(ERROR, "Failed to persist flight metadata");
    flashFull = true;
    flashError = true;
    return;
  }

  resetBufferState();
  metadataLastSavedAddr = metadata.currentWriteAddr;
  metadataDirty = false;
  metadataPersistQueued = false;

  flashInitialized = true;

  Serial.println("DEBUG: flashInitialized=true, flashFull=false");
  logPrint(INFO, "Flash logger initialized, flight #%lu",
           (unsigned long)metadata.flightCounter);
}

uint32_t flashLoggerWrite(const char* msg) {
  writeCallCount++;

  uint32_t messageId = nextMessageId++;

  if (writeCallCount % 100 == 1) {
    Serial.print("DEBUG: flashLoggerWrite call #");
    Serial.print(writeCallCount);
    Serial.print(" msgId=");
    Serial.print(messageId);
    Serial.print(" init=");
    Serial.print(flashInitialized);
    Serial.print(" full=");
    Serial.print(flashFull);
    Serial.print(" err=");
    Serial.println(flashError);
  }

  if (msg == nullptr) {
    return messageId;
  }

  size_t msgLen = strlen(msg);
  bool needsNewline = (msgLen == 0 || msg[msgLen - 1] != '\n');

  if (!flashInitialized || flashFull || flashError) {
    return messageId;
  }

  if (!appendMessageToActive(messageId, msg, msgLen, needsNewline)) {
    uint32_t dropStart = (activeBuffer && activeBuffer->messageCount > 0)
                             ? activeBuffer->firstMessageId
                             : messageId;
    handleDroppedRange(dropStart, messageId);
  }

  return messageId;
}

void flashLoggerFlushCheck() {
  if (!flashInitialized) {
    return;
  }
  if (flashFull || flashError) {
    serviceFlash(false);
    return;
  }
  serviceFlash(true);
}

void flashLoggerErase() {
  currentState = FLASH_ERASING;
  logPrint(INFO, "Erasing flash logs...");

  flashLoggerSyncBlocking(1000);

  Serial.println("DEBUG: Erasing data region (hybrid sector/block)");
  unsigned long startTime = millis();

  const uint32_t dataEndAddr = FLASH_TOTAL_SIZE;
  uint32_t addr = FLASH_DATA_START_ADDR;
  uint32_t sectorsErased = 0;
  uint32_t blocksErased = 0;

  // Step 1: Use 4KB sector erases until we reach the next 64KB boundary
  uint32_t nextBlockBoundary = (addr + (64 * 1024 - 1)) & ~((64 * 1024) - 1);
  if (nextBlockBoundary > dataEndAddr) {
    nextBlockBoundary = dataEndAddr;
  }

  for (; addr < nextBlockBoundary; addr += FLASH_BLOCK_SIZE) {
    if (!qspiEraseBlocking(addr, NRF_QSPI_ERASE_LEN_4KB)) {
      logPrint(ERROR, "Flash erase failed at sector 0x%06lx", (unsigned long)addr);
      currentState = FLASH_IDLE;
      return;
    }
    sectorsErased++;
    if (sectorsErased % 16 == 0) {
      Serial.print("s");
    }
  }

  // Step 2: Erase the bulk of the space with 64KB block erases
  for (; (addr + (64 * 1024)) <= dataEndAddr; addr += 64 * 1024) {
    if (!qspiEraseBlocking(addr, NRF_QSPI_ERASE_LEN_64KB)) {
      logPrint(ERROR, "Flash erase failed at block 0x%06lx", (unsigned long)addr);
      currentState = FLASH_IDLE;
      return;
    }
    blocksErased++;
    if (blocksErased % 4 == 0) {
      Serial.print("B");
    }
  }

  // Step 3: Clean up any remaining tail sectors (should be <64KB)
  for (; addr < dataEndAddr; addr += FLASH_BLOCK_SIZE) {
    if (!qspiEraseBlocking(addr, NRF_QSPI_ERASE_LEN_4KB)) {
      logPrint(ERROR, "Flash erase failed at tail sector 0x%06lx", (unsigned long)addr);
      currentState = FLASH_IDLE;
      return;
    }
    sectorsErased++;
    if (sectorsErased % 16 == 0) {
      Serial.print("s");
    }
  }

  unsigned long elapsedMs = millis() - startTime;
  Serial.println();
  Serial.print("DEBUG: Data erase complete in ");
  Serial.print(elapsedMs);
  Serial.print("ms (");
  Serial.print(blocksErased);
  Serial.print(" blocks, ");
  Serial.print(sectorsErased);
  Serial.println(" sectors)");

  metadata.magic = 0xF117DA7A;
  metadata.flightCounter = 0;
  metadata.currentWriteAddr = FLASH_DATA_START_ADDR;
  metadata.currentFileStartAddr = FLASH_DATA_START_ADDR;
  metadata.numFlights = 0;
  memset(metadata.flightIndex, 0, sizeof(metadata.flightIndex));

  Serial.print("DEBUG: Saving metadata after full erase");
  bool saved = saveMetadataBlocking();
  Serial.print(" - ");
  Serial.println(saved ? "SUCCESS" : "FAILED");

  resetBufferState();
  metadataLastSavedAddr = metadata.currentWriteAddr;
  metadataDirty = false;
  metadataPersistQueued = false;
  flashInitialized = true;
  flashFull = false;
  flashError = false;

  logPrint(INFO, "Flash erased and initialized");
  currentState = FLASH_IDLE;
}

bool flashLoggerIsFull() {
  return flashFull || flashError;
}

bool flashLoggerHasPendingData() {
  if (!flashInitialized) {
    return false;
  }

  if (activeBuffer && activeBuffer->payloadLen > 0) {
    return true;
  }
  if (pendingBuffer != nullptr || inflightBuffer != nullptr) {
    return true;
  }
  if (queuedPaddedBytes > 0) {
    return true;
  }
  if (metadataDirty || metadataPersistQueued || metadataFlushState != META_FLUSH_IDLE) {
    return true;
  }
  if (dataFlushState != DATA_FLUSH_IDLE) {
    return true;
  }
  if (isFlashBusy()) {
    return true;
  }
  return false;
}

uint32_t flashLoggerGetCurrentFlightNumber() {
  return metadata.flightCounter;
}

int flashLoggerGetFileCount() {
  flashLoggerSyncBlocking(FLASH_SYNC_TIMEOUT_DEFAULT_MS);

  int count = metadata.numFlights;
  if (metadata.flightCounter > 0 &&
      (metadata.currentWriteAddr > metadata.currentFileStartAddr ||
       activeBuffer->payloadLen > 0 || pendingBuffer != nullptr || inflightBuffer != nullptr)) {
    count++;
  }
  return count;
}

const char* flashLoggerGetFileName(int index) {
  static char filename[32];

  if (index < (int)metadata.numFlights) {
    snprintf(filename, sizeof(filename), "flight_%03lu.txt",
             (unsigned long)metadata.flightIndex[index].flightNumber);
    return filename;
  }

  if (index == (int)metadata.numFlights && metadata.flightCounter > 0) {
    snprintf(filename, sizeof(filename), "flight_%03lu.txt",
             (unsigned long)metadata.flightCounter);
    return filename;
  }

  return nullptr;
}

uint32_t flashLoggerGetFileSize(int index) {
  flashLoggerSyncBlocking(FLASH_SYNC_TIMEOUT_DEFAULT_MS);

  if (index < (int)metadata.numFlights) {
    return metadata.flightIndex[index].endAddr - metadata.flightIndex[index].startAddr;
  }

  if (index == (int)metadata.numFlights) {
    return metadata.currentWriteAddr - metadata.currentFileStartAddr;
  }

  return 0;
}

bool flashLoggerStartDownload(const char* filename) {
  if (!flashInitialized || currentState != FLASH_IDLE) {
    return false;
  }

  if (!flashLoggerSyncBlocking(FLASH_SYNC_TIMEOUT_DEFAULT_MS)) {
    logPrint(WARNING, "Unable to flush buffers before download");
  }

  uint32_t requestedFlight = 0;
  if (sscanf(filename, "flight_%lu.txt", (unsigned long*)&requestedFlight) != 1) {
    logPrint(ERROR, "Invalid filename format: %s", filename);
    return false;
  }

  downloadBytesSent = 0;
  downloadDebugChunksLogged = 0;

  for (uint32_t i = 0; i < metadata.numFlights; i++) {
    if (metadata.flightIndex[i].flightNumber == requestedFlight) {
      downloadStartAddr = metadata.flightIndex[i].startAddr;
      downloadReadAddr = metadata.flightIndex[i].startAddr;
      downloadEndAddr = metadata.flightIndex[i].endAddr;
      currentState = FLASH_DOWNLOADING;
      downloadFilteredSize = computeFilteredSize(downloadStartAddr, downloadEndAddr);
      logPrint(INFO, "Starting download: %s (%lu bytes)",
               filename, (unsigned long)(downloadEndAddr - downloadStartAddr));
      return true;
    }
  }

  if (requestedFlight == metadata.flightCounter) {
    downloadStartAddr = metadata.currentFileStartAddr;
    downloadReadAddr = metadata.currentFileStartAddr;
    downloadEndAddr = metadata.currentWriteAddr;

    if (downloadReadAddr >= downloadEndAddr) {
      logPrint(WARNING, "Flight has no data (start=%lu, end=%lu)",
               (unsigned long)downloadReadAddr, (unsigned long)downloadEndAddr);
      return false;
    }

    currentState = FLASH_DOWNLOADING;
    downloadFilteredSize = computeFilteredSize(downloadStartAddr, downloadEndAddr);
    logPrint(INFO, "Starting download: %s (%lu bytes) addr 0x%lx-0x%lx",
             filename, (unsigned long)(downloadEndAddr - downloadStartAddr),
             (unsigned long)downloadStartAddr, (unsigned long)downloadEndAddr);
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
    logPrint(DEBUG, "Download complete, total bytes: %lu",
             (unsigned long)(downloadEndAddr - downloadStartAddr));
    return 0;
  }

  if (downloadFilteredSize > 0 && downloadBytesSent >= downloadFilteredSize) {
    return 0;
  }

  size_t produced = 0;
  while (produced == 0 && downloadReadAddr < downloadEndAddr) {
    size_t toRead = downloadEndAddr - downloadReadAddr;
    if (toRead > maxLen) {
      toRead = maxLen;
    }

    if (!qspiRead(downloadReadAddr, buffer, toRead)) {
      logPrint(ERROR, "QSPI read failed at addr 0x%lx", (unsigned long)downloadReadAddr);
      return -1;
    }

    size_t writeIdx = 0;
    for (size_t i = 0; i < toRead; ++i) {
      if (buffer[i] != 0) {
        buffer[writeIdx++] = buffer[i];
      }
    }

    downloadReadAddr += toRead;
    produced = writeIdx;

    if (produced == 0) {
      continue;
    }

    if (downloadFilteredSize > 0) {
      uint32_t remaining = downloadFilteredSize > downloadBytesSent
                               ? (downloadFilteredSize - downloadBytesSent)
                               : 0;
      if (remaining == 0) {
        return 0;
      }
      if (produced > remaining) {
        produced = remaining;
      }
    }

    if (downloadDebugChunksLogged < 4) {
      size_t inspectLen = produced;
      if (inspectLen > 32) {
        inspectLen = 32;
      }
      char asciiBuf[33];
      for (size_t i = 0; i < inspectLen; ++i) {
        char c = (buffer[i] >= 0x20 && buffer[i] <= 0x7E) ? (char)buffer[i] : '.';
        asciiBuf[i] = c;
      }
      asciiBuf[inspectLen] = '\0';

      char hexBuf[3 * 32 + 1];
      for (size_t i = 0; i < inspectLen; ++i) {
        snprintf(&hexBuf[i * 3], 4, "%02X ", buffer[i]);
      }
      hexBuf[inspectLen * 3] = '\0';

      logPrint(DEBUG,
               "Download chunk #%lu addr=0x%06lx len=%lu ascii='%s' hex=%s",
               (unsigned long)(downloadDebugChunksLogged + 1),
               (unsigned long)(downloadReadAddr - toRead),
               (unsigned long)inspectLen,
               asciiBuf,
               hexBuf);
      downloadDebugChunksLogged++;
    }

  }

  if (produced == 0) {
    return 0;
  }

  downloadBytesSent += produced;
  return (int)produced;
}

void flashLoggerStopDownload() {
  currentState = FLASH_IDLE;
  downloadStartAddr = 0;
  downloadReadAddr = 0;
  downloadEndAddr = 0;
  downloadFilteredSize = 0;
  downloadBytesSent = 0;
  downloadDebugChunksLogged = 0;
}

FlashLoggerState flashLoggerGetState() {
  return currentState;
}

static uint32_t computeFilteredSize(uint32_t startAddr, uint32_t endAddr) {
  uint8_t scanBuf[128];
  uint32_t filtered = 0;
  for (uint32_t addr = startAddr; addr < endAddr;) {
    uint32_t remaining = endAddr - addr;
    uint32_t chunk = (remaining < sizeof(scanBuf)) ? remaining : (uint32_t)sizeof(scanBuf);
    if (!qspiRead(addr, scanBuf, chunk)) {
      logPrint(WARNING, "Filtered size scan failed at 0x%06lX", (unsigned long)addr);
      return endAddr - startAddr;
    }
    for (uint32_t i = 0; i < chunk; ++i) {
      if (scanBuf[i] != 0) {
        filtered++;
      }
    }
    addr += chunk;
  }
  return filtered;
}

uint32_t flashLoggerGetActiveDownloadSize() {
  return downloadFilteredSize;
}

// Internal helpers

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
  while (nrfx_qspi_mem_busy_check()) {
    delayMicroseconds(10);
  }
  return true;
}

static bool qspiWriteBlocking(uint32_t addr, const void* data, size_t len) {
  nrfx_err_t err = nrfx_qspi_write(data, len, addr);
  if (err != NRFX_SUCCESS) {
    return false;
  }
  while (nrfx_qspi_mem_busy_check()) {
    delayMicroseconds(10);
  }
  return true;
}

static bool qspiEraseBlocking(uint32_t addr, nrf_qspi_erase_len_t len) {
  nrfx_err_t err = nrfx_qspi_erase(len, addr);
  if (err != NRFX_SUCCESS) {
    return false;
  }
  while (nrfx_qspi_mem_busy_check()) {
    delayMicroseconds(100);
  }
  return true;
}

static bool loadMetadata() {
  if (!qspiRead(FLASH_METADATA_ADDR, &metadata, sizeof(metadata))) {
    return false;
  }

  if (metadata.magic != 0xF117DA7A) {
    return false;
  }

  if (metadata.currentWriteAddr < FLASH_DATA_START_ADDR ||
      metadata.currentWriteAddr >= FLASH_TOTAL_SIZE) {
    return false;
  }

  return true;
}

static bool saveMetadataBlocking() {
  if (!qspiEraseBlocking(FLASH_METADATA_ADDR, NRF_QSPI_ERASE_LEN_4KB)) {
    return false;
  }
  if (!qspiWriteBlocking(FLASH_METADATA_ADDR, &metadata, sizeof(metadata))) {
    return false;
  }
  return true;
}

static void resetBufferState() {
  memset(bufferSlots, 0, sizeof(bufferSlots));
  activeBuffer = &bufferSlots[0];
  pendingBuffer = nullptr;
  inflightBuffer = nullptr;
  freeSlots[0] = &bufferSlots[1];
  freeSlots[1] = &bufferSlots[2];
  freeSlotCount = 2;
  dataFlushState = DATA_FLUSH_IDLE;
  metadataFlushState = META_FLUSH_IDLE;
  inflightChunkSize = 0;
  flushRequested = false;
  queuedPaddedBytes = 0;
  nextSequenceId = 1;
  nextMessageId = 1;
  metadataDirty = false;
  metadataPersistQueued = false;
}

static inline size_t alignToWord(size_t len) {
  return (len + 3UL) & ~0x3UL;
}

static BufferSlot* acquireFreeSlot() {
  if (freeSlotCount == 0) {
    return nullptr;
  }
  BufferSlot* slot = freeSlots[--freeSlotCount];
  slot->payloadLen = 0;
  slot->paddedLen = 0;
  slot->writeOffset = 0;
  slot->targetAddr = 0;
  slot->sequence = 0;
  resetSlotMetadata(slot);
  return slot;
}

static void releaseSlot(BufferSlot* slot) {
  if (!slot) {
    return;
  }
  slot->payloadLen = 0;
  slot->paddedLen = 0;
  slot->writeOffset = 0;
  slot->targetAddr = 0;
  slot->sequence = 0;
  resetSlotMetadata(slot);
  if (freeSlotCount < 2) {
    freeSlots[freeSlotCount++] = slot;
  }
}

static bool tryStageActiveBuffer() {
  if (!activeBuffer || activeBuffer->payloadLen == 0) {
    return false;
  }
  if (pendingBuffer != nullptr) {
    return false;
  }

  BufferSlot* newActive = acquireFreeSlot();
  if (!newActive) {
    return false;
  }

  size_t paddedLen = alignToWord(activeBuffer->payloadLen);
  if (paddedLen == 0) {
    releaseSlot(newActive);
    return false;
  }

  uint32_t targetAddr = metadata.currentWriteAddr + queuedPaddedBytes;
  if (targetAddr + paddedLen > FLASH_TOTAL_SIZE) {
    flashFull = true;
    flushRequested = false;
    releaseSlot(newActive);
    logPrint(WARNING, "Flash full, stopping logging (addr=0x%lx len=%lu)",
             (unsigned long)targetAddr, (unsigned long)paddedLen);
    return false;
  }

  for (size_t i = activeBuffer->payloadLen; i < paddedLen; ++i) {
    activeBuffer->data[i] = 0;
  }

  activeBuffer->paddedLen = paddedLen;
  activeBuffer->writeOffset = 0;
  activeBuffer->targetAddr = targetAddr;
  activeBuffer->sequence = nextSequenceId++;

  queuedPaddedBytes += paddedLen;
  pendingBuffer = activeBuffer;

  activeBuffer = newActive;
  flushRequested = false;
  return true;
}

static void finalizeInflightBuffer() {
  if (!inflightBuffer) {
    return;
  }

  uint32_t completedLen = (uint32_t)inflightBuffer->paddedLen;
  if (metadata.currentWriteAddr < inflightBuffer->targetAddr) {
    metadata.currentWriteAddr = inflightBuffer->targetAddr;
  }
  metadata.currentWriteAddr += completedLen;

  if (queuedPaddedBytes >= completedLen) {
    queuedPaddedBytes -= completedLen;
  } else {
    queuedPaddedBytes = 0;
  }

  logPrint(DEBUG,
           "Flush commit seq=%lu addr=0x%06lX len=%lu newWrite=0x%06lX",
           (unsigned long)inflightBuffer->sequence,
           (unsigned long)inflightBuffer->targetAddr,
           (unsigned long)inflightBuffer->paddedLen,
           (unsigned long)metadata.currentWriteAddr);


  releaseSlot(inflightBuffer);
  inflightBuffer = nullptr;

  metadataDirty = true;
  if ((metadata.currentWriteAddr - metadataLastSavedAddr) >= METADATA_SAVE_INTERVAL) {
    metadataPersistQueued = true;
  }
}

static bool isFlashBusy() {
  return nrfx_qspi_mem_busy_check();
}

static void serviceFlash(bool flushIntent) {
  if (flushIntent && !flashFull && !flashError) {
    flushRequested = true;
  }

  if (flushRequested && !flashFull && !flashError) {
    if (pendingBuffer == nullptr) {
      if (!tryStageActiveBuffer()) {
        // Keep request set so we retry once resources free up
      }
    }
  }

  serviceDataFlush();

  if (metadataPersistQueued && metadataFlushState == META_FLUSH_IDLE &&
      pendingBuffer == nullptr && inflightBuffer == nullptr &&
      !isFlashBusy() && !flashError) {
    metadataFlushState = META_FLUSH_ISSUE_ERASE;
  }

  serviceMetadataFlush();
}

static void serviceDataFlush() {
  switch (dataFlushState) {
    case DATA_FLUSH_IDLE:
      if (inflightBuffer == nullptr && pendingBuffer != nullptr && !isFlashBusy()) {
        inflightBuffer = pendingBuffer;
        pendingBuffer = nullptr;
        dataFlushState = DATA_FLUSH_ISSUE_WRITE;
      }
      break;

    case DATA_FLUSH_ISSUE_WRITE:
      if (inflightBuffer == nullptr) {
        dataFlushState = DATA_FLUSH_IDLE;
        break;
      }
      if (isFlashBusy()) {
        break;
      }
      if (inflightBuffer->writeOffset >= inflightBuffer->paddedLen) {
        dataFlushState = DATA_FLUSH_IDLE;
        break;
      }
      {
        size_t remaining = inflightBuffer->paddedLen - inflightBuffer->writeOffset;
        inflightChunkSize = (remaining > FLASH_PAGE_PROGRAM_SIZE)
                                ? FLASH_PAGE_PROGRAM_SIZE
                                : remaining;
        nrfx_err_t err = nrfx_qspi_write(
            inflightBuffer->data + inflightBuffer->writeOffset,
            inflightChunkSize,
            inflightBuffer->targetAddr + inflightBuffer->writeOffset);
        if (err != NRFX_SUCCESS) {
          flashError = true;
          flashFull = true;
          logPrint(ERROR, "QSPI write failed (err=%d)", err);
          if (queuedPaddedBytes >= inflightBuffer->paddedLen) {
            queuedPaddedBytes -= inflightBuffer->paddedLen;
          } else {
            queuedPaddedBytes = 0;
          }
          releaseSlot(inflightBuffer);
          inflightBuffer = nullptr;
          dataFlushState = DATA_FLUSH_IDLE;
        } else {
          dataFlushState = DATA_FLUSH_WAIT_WRITE;
        }
      }
      break;

    case DATA_FLUSH_WAIT_WRITE:
      if (inflightBuffer == nullptr) {
        dataFlushState = DATA_FLUSH_IDLE;
        break;
      }
      if (isFlashBusy()) {
        break;
      }
      inflightBuffer->writeOffset += inflightChunkSize;
      if (inflightBuffer->writeOffset >= inflightBuffer->paddedLen) {
        finalizeInflightBuffer();
        dataFlushState = DATA_FLUSH_IDLE;
      } else {
        dataFlushState = DATA_FLUSH_ISSUE_WRITE;
      }
      break;
  }
}

static void serviceMetadataFlush() {
  switch (metadataFlushState) {
    case META_FLUSH_IDLE:
      break;

    case META_FLUSH_ISSUE_ERASE:
      if (isFlashBusy()) {
        break;
      }
      {
        nrfx_err_t err = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, FLASH_METADATA_ADDR);
        if (err != NRFX_SUCCESS) {
          logPrint(ERROR, "Metadata erase failed (err=%d)", err);
          flashError = true;
          flashFull = true;
          metadataPersistQueued = true;
          metadataFlushState = META_FLUSH_IDLE;
        } else {
          metadataFlushState = META_FLUSH_WAIT_ERASE;
        }
      }
      break;

    case META_FLUSH_WAIT_ERASE:
      if (!isFlashBusy()) {
        metadataFlushState = META_FLUSH_ISSUE_WRITE;
      }
      break;

    case META_FLUSH_ISSUE_WRITE:
      if (isFlashBusy()) {
        break;
      }
      {
        nrfx_err_t err = nrfx_qspi_write(&metadata, sizeof(metadata), FLASH_METADATA_ADDR);
        if (err != NRFX_SUCCESS) {
          logPrint(ERROR, "Metadata write failed (err=%d)", err);
          flashError = true;
          flashFull = true;
          metadataPersistQueued = true;
          metadataFlushState = META_FLUSH_IDLE;
        } else {
          metadataFlushState = META_FLUSH_WAIT_WRITE;
        }
      }
      break;

    case META_FLUSH_WAIT_WRITE:
      if (!isFlashBusy()) {
        metadataDirty = false;
        metadataPersistQueued = false;
        metadataLastSavedAddr = metadata.currentWriteAddr;
        metadataFlushState = META_FLUSH_IDLE;
      }
      break;
  }
}

static bool isFlushIdle() {
  bool buffersIdle = (pendingBuffer == nullptr && inflightBuffer == nullptr &&
                      (activeBuffer ? activeBuffer->payloadLen == 0 : true));
  bool statesIdle = (dataFlushState == DATA_FLUSH_IDLE &&
                     metadataFlushState == META_FLUSH_IDLE &&
                     !metadataPersistQueued);
  return buffersIdle && statesIdle && !isFlashBusy();
}

static bool flashLoggerSyncBlocking(uint32_t timeoutMs) {
  if (!flashInitialized) {
    return true;
  }

  if (metadataDirty) {
    metadataPersistQueued = true;
  }

  unsigned long start = millis();
  while (true) {
    serviceFlash(true);

    if (isFlushIdle() || flashFull || flashError) {
      return !flashError;
    }

    if (timeoutMs > 0 && (millis() - start) > timeoutMs) {
      return false;
    }

    delayMicroseconds(FLASH_SYNC_POLL_DELAY_US);
  }
}

static void resetSlotMetadata(BufferSlot* slot) {
  if (!slot) {
    return;
  }
  slot->firstMessageId = 0;
  slot->lastMessageId = 0;
  slot->messageCount = 0;
}

static bool appendMessageToActive(uint32_t msgId, const char* msg, size_t msgLen, bool needsNewline) {
  if (!activeBuffer) {
    return false;
  }

  char idPrefix[20];
  int prefixLen = snprintf(idPrefix, sizeof(idPrefix), "#%08lu ", (unsigned long)msgId);
  if (prefixLen <= 0) {
    return false;
  }

  size_t prefixSize = (size_t)prefixLen;
  size_t totalLen = prefixSize + msgLen + (needsNewline ? 1 : 0);

  if (totalLen >= FLASH_LOGGER_BUFFER_SIZE) {
    return false;
  }

  if (activeBuffer->payloadLen + totalLen > FLASH_LOGGER_BUFFER_SIZE) {
    flushRequested = true;
    if (!tryStageActiveBuffer()) {
      return false;
    }
  }

  if (activeBuffer->payloadLen + totalLen > FLASH_LOGGER_BUFFER_SIZE) {
    return false;
  }

  char* dest = activeBuffer->data + activeBuffer->payloadLen;
  memcpy(dest, idPrefix, prefixSize);
  dest += prefixSize;

  if (msgLen > 0) {
    memcpy(dest, msg, msgLen);
    dest += msgLen;
  }

  if (needsNewline) {
    *dest++ = '\n';
  }

  activeBuffer->payloadLen += totalLen;

  if (activeBuffer->messageCount == 0) {
    activeBuffer->firstMessageId = msgId;
  }
  activeBuffer->lastMessageId = msgId;
  if (activeBuffer->messageCount < UINT16_MAX) {
    activeBuffer->messageCount++;
  }

  return true;
}

static void handleDroppedRange(uint32_t dropStartId, uint32_t dropEndId) {
  if (dropStartId == 0 || dropEndId < dropStartId) {
    return;
  }

  if (activeBuffer) {
    activeBuffer->payloadLen = 0;
    activeBuffer->paddedLen = 0;
    activeBuffer->writeOffset = 0;
    activeBuffer->targetAddr = 0;
    activeBuffer->sequence = 0;
    resetSlotMetadata(activeBuffer);
  }

  enqueueDropNotice(dropStartId, dropEndId);
  flushRequested = true;
}

static void enqueueDropNotice(uint32_t dropStartId, uint32_t dropEndId) {
  if (!activeBuffer) {
    return;
  }

  char notice[128];
  unsigned long lost = (unsigned long)(dropEndId - dropStartId + 1);
  snprintf(notice, sizeof(notice),
           "LOG_DROP lost %lu messages (ids %lu-%lu)",
           lost,
           (unsigned long)dropStartId,
           (unsigned long)dropEndId);

  uint32_t dropMsgId = nextMessageId++;
  size_t noticeLen = strlen(notice);
  if (!appendMessageToActive(dropMsgId, notice, noticeLen, true)) {
    // Force reset and try once more; if it still fails give up.
    if (activeBuffer) {
      activeBuffer->payloadLen = 0;
      activeBuffer->paddedLen = 0;
      activeBuffer->writeOffset = 0;
      activeBuffer->targetAddr = 0;
      activeBuffer->sequence = 0;
      resetSlotMetadata(activeBuffer);
    }
    appendMessageToActive(dropMsgId, notice, noticeLen, true);
  }

  Serial.print("WARNING: flash logger dropped messages ");
  Serial.print(dropStartId);
  Serial.print("-");
  Serial.println(dropEndId);
}

static void dumpFirstChunkPreview(uint32_t addr, const BufferSlot* slot) {
  if (!slot) {
    return;
  }

  size_t previewLen = slot->payloadLen;
  if (previewLen == 0) {
    return;
  }
  if (previewLen > 64) {
    previewLen = 64;
  }

  // Snapshot RAM contents
  char asciiBuf[65];
  for (size_t i = 0; i < previewLen; ++i) {
    char c = slot->data[i];
    if (c < 0x20 || c > 0x7E) {
      c = '.';
    }
    asciiBuf[i] = c;
  }
  asciiBuf[previewLen] = '\0';

  char hexLineRam[3 * 64 + 1];
  for (size_t i = 0; i < previewLen; ++i) {
    snprintf(&hexLineRam[i * 3], 4, "%02X ", (uint8_t)slot->data[i]);
  }
  hexLineRam[previewLen * 3] = '\0';

  logPrint(DEBUG,
           "First chunk RAM addr=0x%06lX len=%lu ascii='%s' hex=%s",
           (unsigned long)addr,
           (unsigned long)previewLen,
           asciiBuf,
           hexLineRam);

  uint8_t flashBuf[64];
  if (!qspiRead(addr, flashBuf, previewLen)) {
    logPrint(ERROR, "Preview read failed at 0x%06lX", (unsigned long)addr);
    return;
  }

  char hexLineFlash[3 * 64 + 1];
  for (size_t i = 0; i < previewLen; ++i) {
    snprintf(&hexLineFlash[i * 3], 4, "%02X ", flashBuf[i]);
  }
  hexLineFlash[previewLen * 3] = '\0';

  logPrint(DEBUG,
           "First chunk FLASH addr=0x%06lX len=%lu hex=%s",
           (unsigned long)addr,
           (unsigned long)previewLen,
           hexLineFlash);
}
