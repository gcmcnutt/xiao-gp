#include "main.h"
#include <nrfx_qspi.h>
#include <string.h>
#include <ctype.h>

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
  alignas(4) char data[FLASH_LOGGER_BUFFER_SIZE];
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
static bool loggingSuspended = false;
static alignas(4) uint8_t downloadScratch[FLASH_LOGGER_BUFFER_SIZE];

// Download state
static uint32_t downloadStartAddr = 0;
static uint32_t downloadReadAddr = 0;
static uint32_t downloadEndAddr = 0;
static uint32_t downloadFilteredSize = 0;
static uint32_t downloadBytesSent = 0;
static const uint32_t FNV_OFFSET_BASIS = 2166136261u;
static const uint32_t FNV_PRIME = 16777619u;
static uint32_t writeChecksumCount = 0;
static uint32_t readChecksumCount = 0;
static uint32_t readChecksumHash = FNV_OFFSET_BASIS;
static size_t readSnippetLen = 0;
static char readSnippet[64];
static uint32_t flashWriteChecksumCount = 0;
static uint32_t flashReadChecksumCount = 0;
static uint32_t flashVerifyExpectedHash = 0;
static uint32_t flashVerifyAddr = 0;
static size_t flashVerifyLen = 0;
static size_t flashVerifyOffset = 0;
static bool flashVerifyPending = false;
static alignas(4) uint8_t flashVerifyBuf[FLASH_PAGE_PROGRAM_SIZE];
static bool flashPreviewDumped = false;

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
static uint32_t fnv1aUpdate(uint32_t hash, uint8_t byte);
static void debugPrintChecksum(const char* tag,
                              uint32_t msgIndex,
                              uint32_t hash,
                              const char* snippet);
static void debugDumpBuffer(const char* tag,
                           uint32_t baseAddr,
                           const uint8_t* data,
                           size_t len);
static uint32_t fnv1aUpdate(uint32_t hash, uint8_t byte);
static uint32_t fnv1aUpdate(uint32_t hash, uint8_t byte);
static bool verifyEraseRange(uint32_t startAddr,
                             uint32_t endAddr,
                             uint32_t* firstBadAddr,
                             uint8_t* firstBadValue,
                             bool* readFailure);

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
    .readoc = NRF_QSPI_READOC_FASTREAD,
    .writeoc = NRF_QSPI_WRITEOC_PP,
    .addrmode = NRF_QSPI_ADDRMODE_24BIT,
    .dpmconfig = false,
  },
  .phy_if = {
    .sck_delay = 0,
    .dpmen = false,
    .spi_mode = NRF_QSPI_MODE_0,
    .sck_freq = NRF_QSPI_FREQ_32MDIV8
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
  loggingSuspended = false;

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

  if (!flashInitialized || flashFull || flashError || loggingSuspended) {
    return messageId;
  }

  if (!appendMessageToActive(messageId, msg, msgLen, needsNewline)) {
    uint32_t dropStart = (activeBuffer && activeBuffer->messageCount > 0)
                             ? activeBuffer->firstMessageId
                             : messageId;
    handleDroppedRange(dropStart, messageId);
  } else if (writeChecksumCount < 10) {
    uint32_t hash = FNV_OFFSET_BASIS;
    for (size_t i = 0; i < msgLen; ++i) {
      hash = fnv1aUpdate(hash, (uint8_t)msg[i]);
    }
    if (needsNewline) {
      hash = fnv1aUpdate(hash, (uint8_t) '\n');
    }

    size_t snippetLen = msgLen < 60 ? msgLen : 60;
    char snippet[61];
    for (size_t i = 0; i < snippetLen; ++i) {
      unsigned char ch = (unsigned char)msg[i];
      snippet[i] = isprint(ch) ? (char)ch : '.';
    }
    snippet[snippetLen] = '\0';

    debugPrintChecksum("CHK-WRITE",
                       (uint32_t)messageId,
                       hash,
                       snippet);
    writeChecksumCount++;
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
  if (loggingSuspended) {
    serviceFlash(false);
    return;
  }
  serviceFlash(true);
}

void flashLoggerErase() {
  currentState = FLASH_ERASING;
  logPrint(INFO, "Erasing flash logs...");

  loggingSuspended = true;
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

  uint32_t badAddr = 0;
  uint8_t badValue = 0x00;
  bool readFailure = false;
  if (!verifyEraseRange(FLASH_DATA_START_ADDR,
                        FLASH_TOTAL_SIZE,
                        &badAddr,
                        &badValue,
                        &readFailure)) {
    Serial.print("DEBUG: Erase verify FAILED at 0x");
    Serial.print(badAddr, HEX);
    Serial.print(readFailure ? " (read error)" : " value=0x");
    if (!readFailure) {
      Serial.print(badValue, HEX);
    }
    Serial.println();
    logPrint(ERROR,
             readFailure ? "Erase verify read failed @0x%06lX" : "Erase verify mismatch @0x%06lX val=0x%02X",
             (unsigned long)badAddr,
             (unsigned int)badValue);
    flashError = true;
    flashFull = true;
    currentState = FLASH_IDLE;
    return;
  }

  Serial.println("DEBUG: Erase verify ok (data region all 0xFF)");

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

  loggingSuspended = true;

  if (!flashLoggerSyncBlocking(FLASH_SYNC_TIMEOUT_DEFAULT_MS)) {
    logPrint(WARNING, "Unable to flush buffers before download");
  }

  uint32_t requestedFlight = 0;
  if (sscanf(filename, "flight_%lu.txt", (unsigned long*)&requestedFlight) != 1) {
    logPrint(ERROR, "Invalid filename format: %s", filename);
    loggingSuspended = false;
    return false;
  }

  downloadBytesSent = 0;
  readChecksumCount = 0;
  readChecksumHash = FNV_OFFSET_BASIS;
  readSnippetLen = 0;
  readSnippet[0] = '\0';

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
      loggingSuspended = false;
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
  loggingSuspended = false;
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

    size_t writeIdx = 0;
    for (size_t i = 0; i < toRead; ++i) {
      uint32_t byteAddr = downloadReadAddr + i;
      uint8_t firstSample = 0;
      uint8_t secondSample = 0;
      bool stable = false;

      for (int attempt = 0; attempt < 3 && !stable; ++attempt) {
        while (nrfx_qspi_mem_busy_check()) {
          delayMicroseconds(5);
        }
        if (!qspiRead(byteAddr, &firstSample, 1)) {
          logPrint(ERROR, "QSPI read failed at addr 0x%lx (sample 1)", (unsigned long)byteAddr);
          return -1;
        }
        delayMicroseconds(5);

        while (nrfx_qspi_mem_busy_check()) {
          delayMicroseconds(5);
        }
        if (!qspiRead(byteAddr, &secondSample, 1)) {
          logPrint(ERROR, "QSPI read failed at addr 0x%lx (sample 2)", (unsigned long)byteAddr);
          return -1;
        }

        if (secondSample == firstSample) {
          stable = true;
        } else {
          firstSample = secondSample;
          delayMicroseconds(10);
        }
      }

      uint8_t finalByte = stable ? secondSample : firstSample;
      if (!stable) {
        logPrint(WARNING, "QSPI byte unstable at addr 0x%lx (last=0x%02X)",
                 (unsigned long)byteAddr, finalByte);
      }

      if (finalByte != 0) {
        buffer[writeIdx++] = finalByte;
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

  }

  if (produced == 0) {
    return 0;
  }

  if (readChecksumCount < 10) {
    for (size_t i = 0; i < produced && readChecksumCount < 10; ++i) {
      uint8_t byte = buffer[i];
      readChecksumHash = fnv1aUpdate(readChecksumHash, byte);
      if (byte != '\n' && readSnippetLen < sizeof(readSnippet) - 1) {
        readSnippet[readSnippetLen++] = isprint((unsigned char)byte) ? (char)byte : '.';
      }
      if (byte == '\n') {
        readSnippet[readSnippetLen] = '\0';
        debugPrintChecksum("CHK-READ",
                           readChecksumCount + 1,
                           readChecksumHash,
                           readSnippetLen > 0 ? readSnippet : "<empty>");
        readChecksumCount++;
        readChecksumHash = FNV_OFFSET_BASIS;
        readSnippetLen = 0;
      }
    }
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
  readChecksumCount = 0;
  readChecksumHash = FNV_OFFSET_BASIS;
  readSnippetLen = 0;
  readSnippet[0] = '\0';
  loggingSuspended = false;
}

FlashLoggerState flashLoggerGetState() {
  return currentState;
}

static uint32_t computeFilteredSize(uint32_t startAddr, uint32_t endAddr) {
  alignas(4) uint8_t scanBuf[128];
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

static uint32_t fnv1aUpdate(uint32_t hash, uint8_t byte) {
  hash ^= byte;
  hash *= FNV_PRIME;
  return hash;
}

static void debugPrintChecksum(const char* tag,
                              uint32_t msgIndex,
                              uint32_t hash,
                              const char* snippet) {
  Serial.print("DBG: ");
  Serial.print(tag);
  Serial.print(" msg=");
  Serial.print(msgIndex);
  Serial.print(" sum=0x");
  Serial.print(hash, HEX);
  Serial.print(" text='");
  Serial.print(snippet);
  Serial.println("'");
}

static void debugDumpBuffer(const char* tag,
                           uint32_t baseAddr,
                           const uint8_t* data,
                           size_t len) {
  const size_t bytesPerLine = 16;
  Serial.print("DBG: ");
  Serial.print(tag);
  Serial.print(" @0x");
  Serial.print(baseAddr, HEX);
  Serial.print(" len=");
  Serial.println(len);
  for (size_t offset = 0; offset < len; offset += bytesPerLine) {
    size_t chunk = (offset + bytesPerLine <= len) ? bytesPerLine : (len - offset);
    Serial.print("DBG:   ");
    Serial.print(tag);
    Serial.print("+");
    Serial.print(offset, HEX);
    Serial.print(" ");
    for (size_t i = 0; i < chunk; ++i) {
      uint8_t byte = data[offset + i];
      if (byte < 16) {
        Serial.print('0');
      }
      Serial.print(byte, HEX);
      Serial.print(' ');
    }
    for (size_t i = chunk; i < bytesPerLine; ++i) {
      Serial.print("   ");
    }
    Serial.print(" | ");
    for (size_t i = 0; i < chunk; ++i) {
      uint8_t byte = data[offset + i];
      Serial.print(isprint(byte) ? (char)byte : '.');
    }
    Serial.println();
  }
}

uint32_t flashLoggerGetActiveDownloadSize() {
  return downloadFilteredSize;
}

bool flashLoggerIsSuspended() {
  return loggingSuspended;
}

static bool verifyEraseRange(uint32_t startAddr,
                             uint32_t endAddr,
                             uint32_t* firstBadAddr,
                             uint8_t* firstBadValue,
                             bool* readFailure) {
  if (firstBadAddr) {
    *firstBadAddr = 0;
  }
  if (firstBadValue) {
    *firstBadValue = 0xFF;
  }
  if (readFailure) {
    *readFailure = false;
  }

  alignas(4) uint8_t verifyBuf[FLASH_PAGE_PROGRAM_SIZE];
  for (uint32_t addr = startAddr; addr < endAddr; addr += FLASH_PAGE_PROGRAM_SIZE) {
    size_t remaining = endAddr - addr;
    size_t chunk = remaining < sizeof(verifyBuf) ? remaining : sizeof(verifyBuf);
    if (!qspiRead(addr, verifyBuf, chunk)) {
      if (firstBadAddr) {
        *firstBadAddr = addr;
      }
      if (readFailure) {
        *readFailure = true;
      }
      return false;
    }
    for (size_t i = 0; i < chunk; ++i) {
      if (verifyBuf[i] != 0xFF) {
        if (firstBadAddr) {
          *firstBadAddr = addr + (uint32_t)i;
        }
        if (firstBadValue) {
          *firstBadValue = verifyBuf[i];
        }
        return false;
      }
    }
  }
  return true;
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
  writeChecksumCount = 0;
  flashWriteChecksumCount = 0;
  flashReadChecksumCount = 0;
  flashVerifyExpectedHash = 0;
  flashVerifyAddr = 0;
  flashVerifyLen = 0;
  flashVerifyOffset = 0;
  flashVerifyPending = false;
  flashPreviewDumped = false;
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
        uint32_t chunkAddr = inflightBuffer->targetAddr + inflightBuffer->writeOffset;
        size_t pageOffset = chunkAddr & (FLASH_PAGE_PROGRAM_SIZE - 1);
        size_t spaceInPage = FLASH_PAGE_PROGRAM_SIZE - pageOffset;
        inflightChunkSize = remaining;
        if (inflightChunkSize > spaceInPage) {
          inflightChunkSize = spaceInPage;
        }
        if (inflightChunkSize > FLASH_PAGE_PROGRAM_SIZE) {
          inflightChunkSize = FLASH_PAGE_PROGRAM_SIZE;
        }
        const uint8_t* chunkData = reinterpret_cast<const uint8_t*>(
            inflightBuffer->data + inflightBuffer->writeOffset);
        flashVerifyOffset = inflightBuffer->writeOffset;
        flashVerifyLen = inflightChunkSize;
        flashVerifyAddr = inflightBuffer->targetAddr + flashVerifyOffset;
        flashVerifyExpectedHash = FNV_OFFSET_BASIS;
        for (size_t i = 0; i < flashVerifyLen; ++i) {
          flashVerifyExpectedHash = fnv1aUpdate(flashVerifyExpectedHash, chunkData[i]);
        }
        if (flashWriteChecksumCount < 10) {
          size_t snippetLen = flashVerifyLen < 60 ? flashVerifyLen : (size_t)60;
          char snippet[61];
          size_t idx = 0;
          for (; idx < snippetLen; ++idx) {
            unsigned char ch = chunkData[idx];
            snippet[idx] = isprint(ch) ? (char)ch : '.';
          }
          snippet[idx] = '\0';
          Serial.print("DBG: CHK-FLUSH-WR chunk=");
          Serial.print(flashWriteChecksumCount + 1);
          Serial.print(" addr=0x");
          Serial.print(flashVerifyAddr, HEX);
          Serial.print(" len=");
          Serial.print(flashVerifyLen);
          Serial.print(" sum=0x");
          Serial.print(flashVerifyExpectedHash, HEX);
          Serial.print(" text='");
          Serial.print(snippet);
          Serial.println("'");
          flashWriteChecksumCount++;
        }
        flashVerifyPending = (flashReadChecksumCount < 10);
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
          flashVerifyPending = false;
          flashVerifyExpectedHash = 0;
          flashVerifyLen = 0;
          flashVerifyAddr = 0;
          flashVerifyOffset = 0;
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
      if (flashVerifyPending && flashVerifyLen > 0) {
        // Give the flash a moment to settle before the first verification read.
        delayMicroseconds(5);
        size_t verifyLen = flashVerifyLen;
        if (verifyLen > sizeof(flashVerifyBuf)) {
          verifyLen = sizeof(flashVerifyBuf);
        }
        bool readOk = qspiRead(flashVerifyAddr, flashVerifyBuf, verifyLen);
        if (!readOk) {
          if (flashReadChecksumCount < 10) {
            Serial.print("DBG: CHK-FLUSH-RD chunk=");
            Serial.print(flashReadChecksumCount + 1);
            Serial.print(" addr=0x");
            Serial.print(flashVerifyAddr, HEX);
            Serial.println(" readback failed");
            flashReadChecksumCount++;
          }
        } else {
          const uint8_t* expected = reinterpret_cast<const uint8_t*>(
              inflightBuffer->data + flashVerifyOffset);
          if (!flashPreviewDumped && flashVerifyAddr == FLASH_DATA_START_ADDR) {
            size_t previewLen = verifyLen < 64 ? verifyLen : 64;
            debugDumpBuffer("EXPECT", flashVerifyAddr, expected, previewLen);
            debugDumpBuffer("FLASH", flashVerifyAddr, flashVerifyBuf, previewLen);
            uint32_t beforeAddr = (flashVerifyAddr >= 16) ? (flashVerifyAddr - 16) : 0;
            size_t beforeLen = flashVerifyAddr - beforeAddr;
            if (beforeLen > 0 && beforeLen <= sizeof(flashVerifyBuf)) {
              uint8_t beforeBuf[16];
              size_t readLen = beforeLen < sizeof(beforeBuf) ? beforeLen : sizeof(beforeBuf);
              if (!qspiRead(beforeAddr, beforeBuf, readLen)) {
                Serial.print("DBG: PRE-NRD fail @0x");
                Serial.println(beforeAddr, HEX);
              } else {
                debugDumpBuffer("FLASH-PRE", beforeAddr, beforeBuf, readLen);
              }
            }
            flashPreviewDumped = true;
          }
          bool mismatch = false;
          size_t diffIndex = 0;
          for (size_t i = 0; i < verifyLen; ++i) {
            if (expected[i] != flashVerifyBuf[i]) {
              mismatch = true;
              diffIndex = i;
              break;
            }
          }
          uint32_t readHash = FNV_OFFSET_BASIS;
          for (size_t i = 0; i < verifyLen; ++i) {
            readHash = fnv1aUpdate(readHash, flashVerifyBuf[i]);
          }
          if (flashReadChecksumCount < 10) {
            size_t snippetLen = verifyLen < 60 ? verifyLen : (size_t)60;
            char snippet[61];
            size_t idx = 0;
            for (; idx < snippetLen; ++idx) {
              unsigned char ch = flashVerifyBuf[idx];
              snippet[idx] = isprint(ch) ? (char)ch : '.';
            }
            snippet[idx] = '\0';
            Serial.print("DBG: CHK-FLUSH-RD chunk=");
            Serial.print(flashReadChecksumCount + 1);
            Serial.print(" addr=0x");
            Serial.print(flashVerifyAddr, HEX);
            Serial.print(" len=");
            Serial.print(verifyLen);
            Serial.print(" sum=0x");
            Serial.print(readHash, HEX);
            Serial.print(" match=");
            Serial.print((!mismatch && readHash == flashVerifyExpectedHash) ? "YES" : "NO");
            Serial.print(" text='");
            Serial.print(snippet);
            Serial.println("'");
            if (mismatch || readHash != flashVerifyExpectedHash) {
              Serial.print("DBG: CHK-FLUSH-RD diff@");
              Serial.print(diffIndex);
              Serial.print(" exp=0x");
              Serial.print(expected[diffIndex], HEX);
              Serial.print(" got=0x");
              Serial.println(flashVerifyBuf[diffIndex], HEX);

              // Retry once more after a short wait to see if the flash settles.
              uint8_t retryBuf[64];
              size_t retryLen = verifyLen < sizeof(retryBuf) ? verifyLen : sizeof(retryBuf);
              delayMicroseconds(5);
              if (qspiRead(flashVerifyAddr, retryBuf, retryLen)) {
                uint32_t retryHash = FNV_OFFSET_BASIS;
                for (size_t i = 0; i < retryLen; ++i) {
                  retryHash = fnv1aUpdate(retryHash, retryBuf[i]);
                }
                Serial.print("DBG: CHK-FLUSH-RD2 chunk=");
                Serial.print(flashReadChecksumCount + 1);
                Serial.print(" addr=0x");
                Serial.print(flashVerifyAddr, HEX);
                Serial.print(" len=");
                Serial.print(retryLen);
                Serial.print(" sum=0x");
                Serial.print(retryHash, HEX);
                Serial.print(" text='");
                for (size_t i = 0; i < (retryLen < 60 ? retryLen : (size_t)60); ++i) {
                  unsigned char ch2 = retryBuf[i];
                  Serial.print(isprint(ch2) ? (char)ch2 : '.');
                }
                Serial.println("'");
              }
            }
            flashReadChecksumCount++;
          }
        }
        flashVerifyPending = false;
        flashVerifyExpectedHash = 0;
        flashVerifyLen = 0;
        flashVerifyAddr = 0;
        flashVerifyOffset = 0;
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
