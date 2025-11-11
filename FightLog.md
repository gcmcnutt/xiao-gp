# xiao-gp flight logger

## Hardware & Platform
- **Board**: Seeed XIAO nRF52840 Sense (2MB onboard QSPI flash)
- **Platform**: Arduino mbed-enabled core (PlatformIO target: `xiaoblesense_arduinocore_mbed`)

## Essential Properties
- Use onboard QSPI flash to log flight data
- Write data as close to real time as possible with async flushing
- Call `flush_check()` at end of each `loop()` iteration to flush pending data ASAP (critical for crash recovery)
- Pre-flight preparation: Create new numbered log file on each startup (flight_001.txt, flight_002.txt, etc.)
- If flash becomes full during logging, skip logging remainder of flight
- Enable BLE connectivity when system detects INAV is disarmed

### What can we do via the BLE interface?
- Establish connection from remote computer or phone via Bluetooth Low Energy
- Web page (hosted on client device) uses Web Bluetooth API to connect to XIAO's BLE GATT services
- Download flight logs via BLE file transfer
- Erase all flight logs via BLE command

### Flight Log Format
- Raw text format matching `logPrint()` output from util.cpp
- Format: `#<message_id> <timestamp_ms> <level> <message>\n`
- Example: `#00000001 0012345 i Flight controller initialized`
- Drop marker: `#00001234 LOG_DROP lost 6 messages (ids 1234-1239)` appears whenever buffering pressure forces a discard.
- Logs written alongside existing Serial console output

### Implementation Details

#### Flash Storage (mbed platform) - ✅ IMPLEMENTED
- **Direct nRFx QSPI driver** for hardware access to P25Q16H QSPI flash chip
- **Synchronous operations**: All QSPI read/write/erase calls wait for completion via `nrfx_qspi_mem_busy_check()`
- **Block-based storage** with metadata block (4KB at addr 0x0000)
- **Flight index system** tracks up to 100 flights with start/end addresses
- **File naming**: `flight_NNN.txt` where NNN increments on each boot
- **Metadata persisted** in flash block 0, includes flight counter and file index
- **Auto-erase on first boot**: Detects uninitialized flash, requires manual erase via BLE
- **Automatic reboot** after erase to ensure clean state

#### Write Strategy - ✅ IMPLEMENTED
- **Triple 4KB buffers**: active (producer), pending, and in-flight slots keep writes sequential while the logger stays non-blocking.
- `flashLoggerWrite()` only appends to the active buffer, tags each entry with a monotonic `#00000000` prefix, and never touches QSPI directly; it relies on an explicit `flashLoggerFlushCheck()` (or sync) call to make forward progress.
- **Manual flushing only**: there is intentionally no automatic flush trigger—each end-of-loop invocation of `flashLoggerFlushCheck()` (or a blocking sync) is what advances the state machine.
- `flashLoggerFlushCheck()` (called once per `loop()`) drives a state machine that stages buffers, issues a single 256-byte page program, and polls for completion on subsequent passes.
- **Asynchronous metadata commits**: metadata is marked dirty on each data flush, saved every 4KB, and forced to flash before downloads or file listings.
- **Back-pressure awareness**: when buffers or flash capacity run out, the logger drops the in-flight buffer, immediately queues an internal `LOG_DROP lost X messages (ids A-B)` entry, and the monotonic prefixes make the gap obvious during post-flight analysis.
- **Boot console guard**: the main loop stays idle for ~8 seconds after reset so the USB console can reattach before the first log messages are emitted.

#### BLE Interface (GATT Services) - ✅ IMPLEMENTED
- **Service UUID**: `F1706000-1234-5678-9ABC-DEF012345678`
- **Three characteristics**:
  - **Control** (`F1706001-...`): Write commands (LIST, DL:filename, NEXT, ERASE:ALL, REBOOT)
  - **Data** (`F1706002-...`): Notify with 128-byte chunks during download
  - **Status** (`F1706003-...`): Notify with progress/status messages
- **Request/response flow control**: Browser requests each chunk individually via "NEXT" command
- **Protocol**:
  - `LIST` → `CURRENT:num` → `FILE:name:size` (multiple) → `LIST_DONE`
  - `DL:filename` → `READY:size:chunksize` → browser sends `NEXT` → device sends chunk → repeat
  - `ERASE:ALL` → `ERASING` → (erase + auto-reboot) → `ERASE_DONE`
  - `REBOOT` → `REBOOT` (status) → auto-reboot
- **Chunk size**: 128 bytes (optimized for BLE reliability)
- **No auto-streaming**: Device only sends when browser requests (prevents packet loss)
- **Non-blocking reboot**: 500ms delay allows BLE to send status before reset

#### Web Interface - ✅ IMPLEMENTED
- **Static HTML file** (`web/flight_logger.html`) runs on user's computer/phone
- **Web Bluetooth API** for direct browser-to-device communication
- **Features**:
  - Connect/disconnect to XIAO-GP via BLE
  - List all flight logs (newest first, with file sizes)
  - **Current flight indicator**: Shows "[CURRENT - in progress]" on active log
  - Download any log with progress bar (current file disabled)
  - **Reboot button**: Manually rotate log file (saves current, starts new)
  - Erase all logs with confirmation (triggers auto-reboot)
- **Flow-controlled downloads**: Browser pulls chunks on-demand, no packet loss
- **Progress tracking**: Shows KB received and percentage complete
- **Auto-trimming**: Removes trailing null padding (max 512 bytes)

#### Flight Detection - ⚠️ TODO
- Monitor INAV arming state via MSP link
- Disable BLE advertising when armed (flight mode)
- Enable BLE advertising when disarmed (download mode)
- **Current status**: BLE always enabled

#### Error Handling - ✅ IMPLEMENTED
- **Flash full**: Sets flag, stops logging, continues flight controller operation
- **Flash write failure**: Retries once, then discards buffer to prevent memory buildup
- **QSPI read failure**: Returns error, aborts download with status message
- **Uninitialized flash**: Warns user, disables logging until manual erase
- **BLE disconnection**: Cleans up in-progress transfers automatically

## Current Status

### Working Features ✅
- Flash logging with triple 4KB buffers and non-blocking flush state machine
- Flight counter increments on each boot
- Flight index tracks up to 100 flights
- BLE GATT service for file operations
- Web Bluetooth interface for downloads
- Flow-controlled chunk transfer (128 bytes)
- Automatic reboot after erase
- Progress bars and status messages

### Known Issues 🐛
1. **Downloads may be slow** - 128 byte chunks with request/response adds latency
2. **BLE always enabled** - needs integration with INAV arming state

### Recent Fixes ✅
- **Async flush state machine** - Preallocated buffers feed page-program-sized writes while the main loop remains responsive; metadata commits now piggyback on the same scheduler.
- Fixed download completion trigger (request final chunk after reaching expected size)
- Implemented non-blocking reboot (500ms delay, allows BLE to send status)
- Added REBOOT button for manual log rotation
- Prevent downloading current (in-progress) flight file
- Send current flight number during LIST to identify active log

### Testing Needed 🧪
- [ ] Verify complete file integrity (all bytes received)
- [ ] Test with multiple flights (3-5 flights)
- [ ] Verify flash erase + reboot cycle
- [ ] Test behavior when flash fills up
- [ ] Validate crash recovery (partial flush preservation)
