#include <main.h>
#include <GP/autoc/aircraft_state.h>
#include <embedded_pathgen_selector.h>
#include <GP/autoc/gp_evaluator_embedded.h>
#include <gp_program.h>
#include <mbed.h>
#include <vector>
#include <cmath>
#include <algorithm>

MSP msp;

State state;

// GP Rabbit Path Following System
static EmbeddedPathSelector path_generator;
static std::vector<Path> flight_path;
static AircraftState aircraft_state;
static Path gp_path_segment; // Single GP-compatible path segment for evaluator

// Test origin anchoring (captured at autoc enable; aircraft_state is expressed relative to this origin)
static gp_vec3 test_origin_offset(0.0f, 0.0f, 0.0f);
static bool test_origin_set = false;

// GP Control Timing and State
static unsigned long rabbit_start_time = 0;
static volatile bool rabbit_active = false;
static int current_path_index = 0;
static int selected_path_index = 0;  // Path selected from RC channel (0-5)
static bool servo_reset_required = false;

// MSP Control Output Caching and scheduling
static volatile int cached_roll_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static volatile int cached_pitch_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static volatile int cached_throttle_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static volatile uint32_t cached_cmd_sequence = 0;
static volatile uint32_t cached_eval_start_us = 0;
static volatile uint32_t cached_eval_complete_us = 0;
static volatile bool mspBusLocked = false;
static volatile bool mspSendPending = false;
static mbed::Ticker mspSendTicker;
static bool mspSendTickerRunning = false;

static constexpr gp_scalar GP_RAD_TO_DEG = static_cast<gp_scalar>(57.2957795f);
static constexpr gp_scalar GP_HALF_PI = static_cast<gp_scalar>(1.57079633f);
static constexpr gp_scalar GP_INV_1000 = static_cast<gp_scalar>(0.001f);

#define MSP_SEND_LOG_CAPACITY 256
struct MspSendLogEntry
{
  uint32_t sendTimeUs;
  uint32_t evalStartUs;
  uint32_t evalEndUs;
  uint32_t sequence;
  bool sequenceChanged;
};
static MspSendLogEntry mspSendLog[MSP_SEND_LOG_CAPACITY];
static volatile uint16_t mspSendLogWriteIndex = 0;
static volatile uint16_t mspSendLogReadIndex = 0;
static volatile uint16_t mspSendLogCount = 0;
static volatile uint32_t lastLoggedSequenceForStats = 0;

// Aircraft state tracking for position/velocity calculation
static gp_vec3 last_valid_position(0.0f, 0.0f, 0.0f);
static bool have_valid_position = false;
static bool was_system_armed = false;

// Safety timeout for single GP test run (60 seconds max per run)
#define GP_MAX_SINGLE_RUN_MSEC (60 * 1000)
#define MSP_BUS_LOCK_TIMEOUT_USEC (MSP_REPLY_TIMEOUT_MSEC * 2000UL)

// Forward declarations for MSP scheduling helpers
static void updateCachedCommands(int roll, int pitch, int throttle, uint32_t evalStartUs);
static bool tryLockMspBusFromTask();
static bool lockMspBusBlockingFromTask();
static bool releaseMspBusFromTask();
static void servicePendingMspSendFromTask();
static void attemptImmediateMspSendFromTask();
static bool performMspRequest(uint16_t command, void *buffer, size_t size);
static bool tryLockMspBusFromIsr();
static void unlockMspBusFromIsr();
static void performMspSendLocked();
static void mspSendTimerHandler();
static void startMspSendTicker();
static void stopMspSendTicker();
static void recordMspSendEvent();
static void resetMspSendStats();
static void logMspSendStats();

static void resetPositionHistory()
{
  last_valid_position = gp_vec3(0.0f, 0.0f, 0.0f);
  have_valid_position = false;
}

static void stopAutoc(const char *reason, bool requireServoReset)
{
  bool wasAutoc = state.autoc_enabled;
  bool wasRabbit = rabbit_active;
  bool latchBefore = servo_reset_required;
  bool tickerWasRunning = mspSendTickerRunning;

  state.autoc_enabled = false;
  rabbit_active = false;
  test_origin_set = false;
  test_origin_offset = gp_vec3(0.0f, 0.0f, 0.0f);
  state.autoc_countdown = 0;
  resetPositionHistory();
  analogWrite(GREEN_PIN, 255);

  if (requireServoReset)
  {
    servo_reset_required = true;
  }

  if (tickerWasRunning)
  {
    stopMspSendTicker();
  }

  if (wasAutoc || wasRabbit || (requireServoReset && !latchBefore))
  {
    logPrint(INFO, "GP Control: Autoc disabled (%s) - pilot has control", reason);
  }

  if (wasRabbit)
  {
    logMspSendStats();
  }
}

static gp_vec3 neuVectorToNedMeters(const int32_t vec_cm[3])
{
  const gp_scalar inv100 = static_cast<gp_scalar>(0.01f);
  gp_scalar north = static_cast<gp_scalar>(vec_cm[0]) * inv100;
  gp_scalar east = static_cast<gp_scalar>(vec_cm[1]) * inv100;
  gp_scalar down = -static_cast<gp_scalar>(vec_cm[2]) * inv100;
  return gp_vec3(north, east, down);
}

static gp_quat neuQuaternionToNed(const float q[4])
{
  // INAV sends body->earth quaternion (confirmed via bench testing).
  // GP expects earth->body, so we take the conjugate.
  // Bench tests show ALL vector components need sign flip.
  gp_quat attitude(q[0], -q[1], -q[2], -q[3]);  // Full conjugate
  if (attitude.norm() == 0.0f)
  {
    return gp_quat::Identity();
  }
  attitude.normalize();
  return attitude;
}

static void mspUpdateGPControl()
{
  // Check for disarm or failsafe conditions before GP control
  // Only check if we're currently enabled to avoid repeat logging
  if (state.autoc_enabled && rabbit_active)
  {
    bool isArmed = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_ARM);
    bool isFailsafe = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_FAILSAFE);

    if (!isArmed || isFailsafe)
    {
      unsigned long test_run_duration = millis() - rabbit_start_time;
      if (isFailsafe)
      {
        logPrint(INFO, "GP Control: INAV failsafe activated (%.1fs) - disabling autoc", test_run_duration * GP_INV_1000);
      }
      else
      {
        logPrint(INFO, "GP Control: Aircraft disarmed (%.1fs) - disabling autoc", test_run_duration * GP_INV_1000);
      }
      stopAutoc(isFailsafe ? "failsafe" : "disarmed", true);
      return;
    }
  }

  // GP rabbit path following control - only if autoc is still enabled
  if (!state.autoc_enabled || !rabbit_active || flight_path.empty())
  {
    return; // Exit early if GP control has been disabled
  }

  // Proceed with GP control
  unsigned long current_time = millis();
  unsigned long elapsed_msec = current_time - rabbit_start_time;

  // Check termination conditions
  if (elapsed_msec > GP_MAX_SINGLE_RUN_MSEC)
  {
    logPrint(INFO, "GP Control: Test run timeout (%.1fs) - stopping rabbit", elapsed_msec * GP_INV_1000);
    stopAutoc("timeout", true);
    return;
  }

  // Find current path segment based on elapsed time since autoc enabled
  current_path_index = getRabbitPathIndex(elapsed_msec);

  // End of path check
  if (current_path_index >= (int)flight_path.size() - 1)
  {
    logPrint(INFO, "GP Control: End of path reached (%.1fs) - stopping rabbit", elapsed_msec * GP_INV_1000);
    stopAutoc("path complete", true);
    return;
  }

  // GP evaluation - calculate new commands
  if (current_path_index < (int)flight_path.size())
  {
    gp_path_segment = flight_path[current_path_index];

    // Set current path index for GP evaluation
    aircraft_state.setThisPathIndex(current_path_index);

    // Call generated GP program directly (aircraft_state is updated continuously in mspUpdateState)
    SinglePathProvider pathProvider(gp_path_segment, aircraft_state.getThisPathIndex());
    uint32_t eval_start_us = micros();
    
    // Calculate craft-to-target vector in world frame
    gp_vec3 craftToTarget = gp_path_segment.start - aircraft_state.getPosition();
    gp_scalar debug_getdtheta = evaluateGPOperator(GETDTHETA, pathProvider, aircraft_state, nullptr, 0, 0.0f);
    gp_scalar debug_getdphi = evaluateGPOperator(GETDPHI, pathProvider, aircraft_state, nullptr, 0, 0.0f);
    gp_scalar debug_getalpha = evaluateGPOperator(GETALPHA, pathProvider, aircraft_state, nullptr, 0, 0.0f);
    gp_scalar debug_getbeta = evaluateGPOperator(GETBETA, pathProvider, aircraft_state, nullptr, 0, 0.0f);
    gp_scalar debug_getdhome = evaluateGPOperator(GETDHOME, pathProvider, aircraft_state, nullptr, 0, 0.0f);

    // GP inputs relative to rabbit
    logPrint(INFO,
             "GP Input: idx=%d rabbit=[%.1f,%.1f,%.1f] vec=[%.1f,%.1f,%.1f] alpha[0]=%.3f beta[0]=%.3f dtheta[0]=%.3f dphi[0]=%.3f dhome[0]=%.3f relvel=%.2f",
             current_path_index,
             gp_path_segment.start.x(), gp_path_segment.start.y(), gp_path_segment.start.z(),
             craftToTarget.x(), craftToTarget.y(), craftToTarget.z(),
             debug_getalpha,
             debug_getbeta,
             debug_getdtheta,
             debug_getdphi,
             debug_getdhome,
             aircraft_state.getRelVel());

    // Capture temporal history before GP evaluation (for GETDPHI_PREV, GETDTHETA_PREV, etc.)
    aircraft_state.recordErrorHistory(debug_getdphi, debug_getdtheta, millis());

    generatedGPProgram(pathProvider, aircraft_state, 0.0f);

    // Convert GP-controlled aircraft commands to MSP RC values and cache them
    int roll_cmd = convertRollToMSPChannel(aircraft_state.getRollCommand());
    int pitch_cmd = convertPitchToMSPChannel(aircraft_state.getPitchCommand());
    int throttle_cmd = convertThrottleToMSPChannel(aircraft_state.getThrottleCommand());
    updateCachedCommands(roll_cmd, pitch_cmd, throttle_cmd, eval_start_us);

    logPrint(INFO, "GP Output: rc=[%d,%d,%d]", roll_cmd, pitch_cmd, throttle_cmd);
  }
}

void msplinkSetup()
{
  // Initialize MSPLink input serial1 port
  Serial1.begin(115200);
  msp.begin(Serial1, MSP_REPLY_TIMEOUT_MSEC);
  logPrint(INFO, "MSPLink Reader Started");

  // set 'valid' values for now
  for (int i = 0; i < MSP_MAX_SUPPORTED_CHANNELS; i++)
  {
    state.command_buffer.channel[i] = MSP_DEFAULT_CHANNEL_VALUE;
  }

  // Start periodic MSP send timer
  startMspSendTicker();
  resetMspSendStats();
}

void mspUpdateState()
{
  state.resetState();
  state.setAsOfMsec(millis());

  // get status
  state.status_valid = performMspRequest(MSP_STATUS, &state.status, sizeof(state.status));
  if (!state.status_valid)
  {
    stopAutoc("MSP status failure", true);
    logPrint(ERROR, "*** CRITICAL: Failed to get MSP_STATUS - aborting MSP update cycle");
    return; // Critical error - can't continue without status
  }

  // Local state (position / velocity / quaternion)
  state.local_state_valid = performMspRequest(MSP2_INAV_LOCAL_STATE, &state.local_state, sizeof(state.local_state));
  if (state.local_state_valid)
  {
    // Convert INAV timestamp from microseconds to milliseconds
    state.inavSampleTimeMsec = state.local_state.timestamp_us / 1000;
  }
  // Local state already provides quaternion data; if unavailable, orientation will be held from previous sample
  if (!state.local_state_valid && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc("MSP local state failure", true);
  }

  // RC channels
  state.rc_valid = performMspRequest(MSP_RC, &state.rc, sizeof(state.rc));
  if (!state.rc_valid && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc("MSP RC failure", true);
  }

  // Sample path selector channel every cycle (for pre-flight validation and GP State logging)
  int pathSelectorRcValue = MSP_DEFAULT_CHANNEL_VALUE;
  int pathSelectorIndex = 0;  // Default to path 0 (StraightAndLevel)

  if (state.rc_valid) {
    pathSelectorRcValue = state.rc.channelValue[MSP_PATH_SELECT_CHANNEL];
    // Map 1000-2000 → 0-5 (6-position switch)
    // Each position gets ~167μs range: 1000, 1200, 1400, 1600, 1800, 2000
    int clamped = constrain(pathSelectorRcValue, 1000, 2000);
    pathSelectorIndex = min(5, (clamped - 1000) * 6 / 1001);  // Scale to 0-5
  }

  // ok, let's see what we fetched and updated.
  // first, check if we have a valid status
  bool isArmed = state.status_valid && state.status.flightModeFlags & (1 << MSP_MODE_ARM);
  if (isArmed)
  {
    analogWrite(BLUE_PIN, 0);
  }
  else
  {
    analogWrite(BLUE_PIN, 255);
  }

  // Manage flash logging and BLE state based on arm transitions
  if (isArmed != was_system_armed)
  {
    if (isArmed)
    {
      blueToothSetEnabled(false);
      if (!flashLoggerBeginFlight())
      {
        logPrint(ERROR, "Flash logger failed to start new flight on arm");
      }
    }
    else
    {
      flashLoggerEndFlight();
      blueToothSetEnabled(true);
    }
    was_system_armed = isArmed;
  }

  // then, check the servo channel to see if can auto-enable
  bool hasServoActivation = state.rc_valid && state.rc.channelValue[MSP_ARM_CHANNEL] > MSP_ARMED_THRESHOLD;

  if (!isArmed && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc("disarmed", true);
  }

  bool hadServoLatch = servo_reset_required;
  if (!hasServoActivation)
  {
    if (hadServoLatch)
    {
      logPrint(INFO, "GP Control: Servo reset detected - autoc re-arm allowed");
    }
    servo_reset_required = false;
    state.autoc_countdown = 0;
  }
  else if (!servo_reset_required && isArmed)
  {
    state.autoc_countdown++;
  }
  else
  {
    state.autoc_countdown = 0;
  }

  // Countdown timer logic to determine autoc_enabled state
  bool new_autoc_enabled = state.autoc_countdown > MSP_ARM_CYCLE_COUNT;

  // Handle state transitions
  if (new_autoc_enabled && !state.autoc_enabled)
  {
    resetPositionHistory();

    if (state.local_state_valid)
    {
      test_origin_offset = neuVectorToNedMeters(state.local_state.pos); // capture absolute INAV position at enable
      test_origin_set = true;

      // Use cached path selector value from cycle sampling
      int pathIndex = pathSelectorIndex;
      selected_path_index = pathIndex;  // Store armed path index for reference

      // Generate selected path with seed for reproducibility
      const char* pathNames[] = {
        "StraightAndLevel",
        "SpiralClimb",
        "HorizontalFigureEight",
        "FortyFiveDegreeAngledLoop",
        "HighPerchSplitS",
        "SeededRandomB"
      };

      uint32_t generation_start_us = micros();
      // Generate path at canonical origin (0,0,0) - craft is already at virtual (0,0,0)
      path_generator.generatePath(pathIndex, 0.0f, EMBEDDED_PATH_SEED);
      uint32_t generation_duration_us = micros() - generation_start_us;

      path_generator.copyToVector(flight_path);

      // Log path generation summary
      logPrint(INFO, "Path armed: %d=%s, %d/%d segments, origin=(0,0,0), seed=%u, time=%.1fms",
               pathIndex, pathNames[pathIndex], (int)flight_path.size(), MAX_EMBEDDED_PATH_SEGMENTS,
               EMBEDDED_PATH_SEED, generation_duration_us / 1000.0f);

      if (path_generator.wasTruncated()) {
        logPrint(WARNING, "*** Path was TRUNCATED at MAX_EMBEDDED_PATH_SEGMENTS=%d ***",
                 MAX_EMBEDDED_PATH_SEGMENTS);
      }

      rabbit_start_time = millis();
      rabbit_active = true;
      resetMspSendStats();
      startMspSendTicker();
      current_path_index = 0;

      state.autoc_enabled = true;
      servo_reset_required = false;
      analogWrite(GREEN_PIN, 0);
      logPrint(INFO, "GP Control: Switch enabled - origin NED=[%.2f, %.2f, %.2f] - program=%s",
               test_origin_offset.x(), test_origin_offset.y(), test_origin_offset.z(),
               generatedGPProgramSource);
    }
    else
    {
      stopAutoc("missing local state", true);
      logPrint(ERROR, "*** FATAL: No valid local state available for GP control - cannot enable autoc");
    }
  }
  else if (!new_autoc_enabled && state.autoc_enabled)
  {
    if (rabbit_active)
    {
      unsigned long test_run_duration = millis() - rabbit_start_time;
      logPrint(INFO, "GP Control: Switch disabled (%.1fs) - stopping test run", test_run_duration * GP_INV_1000);
    }
    if (!isArmed)
    {
      stopAutoc("disarmed", true);
    }
    else if (!hasServoActivation)
    {
      stopAutoc("servo switch", false);
    }
    else
    {
      stopAutoc("autoc cancelled", true);
    }
  }

  // Update aircraft state on every MSP cycle for continuous position/velocity tracking
  convertMSPStateToAircraftState(aircraft_state);

  // Log raw vs GP-relative state for correlation (no inference beyond origin offset)
  gp_vec3 pos_raw = gp_vec3::Zero();
  gp_vec3 vel_raw = gp_vec3::Zero();
  if (state.local_state_valid)
  {
    pos_raw = neuVectorToNedMeters(state.local_state.pos);
    vel_raw = neuVectorToNedMeters(state.local_state.vel);
  }
  else if (have_valid_position)
  {
    pos_raw = last_valid_position;
  }
  gp_vec3 pos_rel = aircraft_state.getPosition();
  gp_vec3 vel_rel = aircraft_state.getVelocity();
  gp_quat q = aircraft_state.getOrientation();
  if (q.norm() > 0.0f)
  {
    q.normalize();
  }
  bool armed = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_ARM);
  bool failsafe = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_FAILSAFE);

  logPrint(INFO,
           "GP State: pos_raw=[%.2f,%.2f,%.2f] pos=[%.2f,%.2f,%.2f] vel=[%.2f,%.2f,%.2f] quat=[%.3f,%.3f,%.3f,%.3f] armed=%s fs=%s servo=%s autoc=%s rabbit=%s path=%d",
           pos_raw.x(), pos_raw.y(), pos_raw.z(),
           pos_rel.x(), pos_rel.y(), pos_rel.z(),
           vel_rel.x(), vel_rel.y(), vel_rel.z(),
           q.w(), q.x(), q.y(), q.z(),
           armed ? "Y" : "N",
           failsafe ? "Y" : "N",
           hasServoActivation ? "Y" : "N",
           state.autoc_enabled ? "Y" : "N",
           rabbit_active ? "Y" : "N",
           pathSelectorIndex);

  // Update GP control and cache commands when enabled
  mspUpdateGPControl();
}

void mspSetControls()
{
  attemptImmediateMspSendFromTask();
}

static void updateCachedCommands(int roll, int pitch, int throttle, uint32_t evalStartUs)
{
  uint32_t evalEndUs = micros();
  noInterrupts();
  cached_roll_cmd = roll;
  cached_pitch_cmd = pitch;
  cached_throttle_cmd = throttle;
  cached_eval_start_us = evalStartUs;
  cached_eval_complete_us = evalEndUs;
  cached_cmd_sequence++;
  interrupts();
}

static bool tryLockMspBusFromTask()
{
  bool locked = false;
  noInterrupts();
  if (!mspBusLocked)
  {
    mspBusLocked = true;
    locked = true;
  }
  interrupts();
  return locked;
}

static bool lockMspBusBlockingFromTask()
{
  unsigned long start = micros();
  while (!tryLockMspBusFromTask())
  {
    if (micros() - start >= MSP_BUS_LOCK_TIMEOUT_USEC)
    {
      logPrint(ERROR, "MSP bus lock timeout - MSP command %s", state.autoc_enabled ? "during autoc" : "idle");
      return false;
    }
    delayMicroseconds(50);
  }
  return true;
}

static bool releaseMspBusFromTask()
{
  bool pendingSend = false;
  noInterrupts();
  mspBusLocked = false;
  pendingSend = mspSendPending;
  mspSendPending = false;
  interrupts();
  return pendingSend;
}

static void servicePendingMspSendFromTask()
{
  while (rabbit_active)
  {
    if (!tryLockMspBusFromTask())
    {
      noInterrupts();
      mspSendPending = true;
      interrupts();
      return;
    }

    performMspSendLocked();
    bool morePending = releaseMspBusFromTask();
    if (!morePending)
    {
      return;
    }
  }
}

static void attemptImmediateMspSendFromTask()
{
  if (!rabbit_active)
  {
    return;
  }

  if (!tryLockMspBusFromTask())
  {
    noInterrupts();
    mspSendPending = true;
    interrupts();
    return;
  }

  performMspSendLocked();
  bool pending = releaseMspBusFromTask();
  if (pending)
  {
    servicePendingMspSendFromTask();
  }
}

static bool performMspRequest(uint16_t command, void *buffer, size_t size)
{
  if (!lockMspBusBlockingFromTask())
  {
    return false;
  }
  bool success = msp.request(command, buffer, size);
  bool pending = releaseMspBusFromTask();
  if (pending)
  {
    servicePendingMspSendFromTask();
  }
  return success;
}

static bool tryLockMspBusFromIsr()
{
  if (mspBusLocked)
  {
    return false;
  }
  mspBusLocked = true;
  return true;
}

static void unlockMspBusFromIsr()
{
  mspBusLocked = false;
}

static void performMspSendLocked()
{
  state.command_buffer.channel[0] = cached_roll_cmd;
  state.command_buffer.channel[1] = cached_pitch_cmd;
  state.command_buffer.channel[2] = cached_throttle_cmd;
  msp.send(MSP_SET_RAW_RC, &state.command_buffer, sizeof(state.command_buffer));
  recordMspSendEvent();
}

static void mspSendTimerHandler()
{
  if (!rabbit_active)
  {
    return;
  }

  if (!tryLockMspBusFromIsr())
  {
    mspSendPending = true;
    return;
  }

  performMspSendLocked();
  unlockMspBusFromIsr();
}

static void recordMspSendEvent()
{
  if (!rabbit_active)
  {
    return;
  }

  uint32_t sendTime = micros();
  noInterrupts();
  uint32_t sequence = cached_cmd_sequence;
  uint32_t evalStart = cached_eval_start_us;
  uint32_t evalEnd = cached_eval_complete_us;
  bool sequenceChanged = sequence != lastLoggedSequenceForStats;
  lastLoggedSequenceForStats = sequence;

  uint16_t idx = mspSendLogWriteIndex;
  mspSendLog[idx].sendTimeUs = sendTime;
  mspSendLog[idx].evalStartUs = evalStart;
  mspSendLog[idx].evalEndUs = evalEnd;
  mspSendLog[idx].sequence = sequence;
  mspSendLog[idx].sequenceChanged = sequenceChanged;

  mspSendLogWriteIndex = (idx + 1) % MSP_SEND_LOG_CAPACITY;
  if (mspSendLogCount < MSP_SEND_LOG_CAPACITY)
  {
    mspSendLogCount++;
  }
  else
  {
    mspSendLogReadIndex = (mspSendLogReadIndex + 1) % MSP_SEND_LOG_CAPACITY;
  }
  interrupts();
}

static void resetMspSendStats()
{
  noInterrupts();
  mspSendLogWriteIndex = 0;
  mspSendLogReadIndex = 0;
  mspSendLogCount = 0;
  lastLoggedSequenceForStats = cached_cmd_sequence;
  interrupts();
}

static void logMspSendStats()
{
  uint16_t count;
  uint16_t readIdx;
  noInterrupts();
  count = mspSendLogCount;
  readIdx = mspSendLogReadIndex;
  interrupts();

  if (count == 0)
  {
    logPrint(INFO, "MSP TX stats: no transmissions recorded");
    return;
  }

  gp_scalar intervalSumMs = 0.0f;
  gp_scalar intervalMinMs = 1e9f;
  gp_scalar intervalMaxMs = 0.0f;
  uint32_t intervalCount = 0;
  uint32_t lateIntervals = 0;
  const uint32_t desiredIntervalUs = MSP_SEND_INTERVAL_MSEC * 1000UL;
  const uint32_t lateThresholdUs = desiredIntervalUs + 20000UL; // allow 20ms slack (70ms total)

  gp_scalar latencyStartSumMs = 0.0f;
  gp_scalar latencyEndSumMs = 0.0f;
  gp_scalar latencyStartMinMs = 1e9f;
  gp_scalar latencyStartMaxMs = 0.0f;
  gp_scalar latencyEndMinMs = 1e9f;
  gp_scalar latencyEndMaxMs = 0.0f;
  uint32_t latencySamples = 0;

  uint32_t prevSend = 0;

  for (uint16_t i = 0; i < count; ++i)
  {
    uint16_t idx = (readIdx + i) % MSP_SEND_LOG_CAPACITY;
    const MspSendLogEntry &entry = mspSendLog[idx];

    if (i > 0)
    {
      uint32_t delta = entry.sendTimeUs - prevSend;
      gp_scalar deltaMs = static_cast<gp_scalar>(delta) * GP_INV_1000;
      intervalSumMs += deltaMs;
      intervalMinMs = std::min(intervalMinMs, deltaMs);
      intervalMaxMs = std::max(intervalMaxMs, deltaMs);
      intervalCount++;
      if (delta > lateThresholdUs)
      {
        lateIntervals++;
      }
    }
    prevSend = entry.sendTimeUs;

    if (entry.sequenceChanged && entry.evalStartUs != 0 && entry.evalEndUs != 0)
    {
      gp_scalar startLatencyMs = static_cast<gp_scalar>(entry.sendTimeUs - entry.evalStartUs) * GP_INV_1000;
      gp_scalar endLatencyMs = static_cast<gp_scalar>(entry.sendTimeUs - entry.evalEndUs) * GP_INV_1000;
      latencyStartSumMs += startLatencyMs;
      latencyEndSumMs += endLatencyMs;
      latencyStartMinMs = std::min(latencyStartMinMs, startLatencyMs);
      latencyStartMaxMs = std::max(latencyStartMaxMs, startLatencyMs);
      latencyEndMinMs = std::min(latencyEndMinMs, endLatencyMs);
      latencyEndMaxMs = std::max(latencyEndMaxMs, endLatencyMs);
      latencySamples++;
    }
  }

  gp_scalar intervalAvgMs = intervalCount ? intervalSumMs / intervalCount : 0.0f;
  if (intervalCount == 0)
  {
    intervalMinMs = 0.0f;
  }

  logPrint(INFO,
           "MSP TX stats: sends=%u intervals(ms) min=%.1f avg=%.1f max=%.1f late>%ums=%u",
           count,
           intervalMinMs,
           intervalAvgMs,
           intervalMaxMs,
           (int)(lateThresholdUs / 1000),
           lateIntervals);

  if (latencySamples > 0)
  {
    gp_scalar latencyStartAvgMs = latencyStartSumMs / latencySamples;
    gp_scalar latencyEndAvgMs = latencyEndSumMs / latencySamples;
    logPrint(INFO,
             "MSP latency: samples=%u start(ms) min=%.1f avg=%.1f max=%.1f end(ms) min=%.1f avg=%.1f max=%.1f",
             latencySamples,
             latencyStartMinMs,
             latencyStartAvgMs,
             latencyStartMaxMs,
             latencyEndMinMs,
             latencyEndAvgMs,
             latencyEndMaxMs);
  }
  else
  {
    logPrint(INFO, "MSP latency: no GP evaluations recorded");
  }

  resetMspSendStats();
}

// Convert MSP state data to AircraftState for GP evaluator
void convertMSPStateToAircraftState(AircraftState &aircraftState)
{
  if (!state.local_state_valid)
  {
    if (!have_valid_position)
    {
      return;
    }
  }

  gp_vec3 position_raw = have_valid_position ? last_valid_position : gp_vec3(0.0f, 0.0f, 0.0f);
  gp_vec3 position_rel = position_raw;
  gp_vec3 velocity = gp_vec3::Zero();
  gp_quat orientation = aircraftState.getOrientation();

  if (state.local_state_valid)
  {
    position_raw = neuVectorToNedMeters(state.local_state.pos);
    velocity = neuVectorToNedMeters(state.local_state.vel);
    orientation = neuQuaternionToNed(state.local_state.q);

    last_valid_position = position_raw;
    have_valid_position = true;
  }
  if (test_origin_set)
  {
    position_rel = position_raw - test_origin_offset; // express aircraft_state in virtual-origin frame
  }
  else
  {
    position_rel = position_raw;
  }

  // Paths now generate at canonical (0,0,0); craft already at virtual (0,0,0) when armed
  // If no new quaternion data, retain previous orientation from aircraft state

  gp_scalar speed_magnitude = velocity.norm();

  aircraftState.setPosition(position_rel);
  aircraftState.setOrientation(orientation);
  aircraftState.setVelocity(velocity);
  aircraftState.setRelVel(speed_magnitude);
  aircraftState.setSimTimeMsec(state.asOfMsec);
}

// Find path index based on elapsed time since autoc enabled
int getRabbitPathIndex(unsigned long elapsed_msec)
{
  if (flight_path.empty())
    return 0;

  // Linear scan from current point to find the path segment just beyond where we are in time
  // Since time only moves forward, start from the current index to avoid redundant searches
  for (size_t i = current_path_index; i < flight_path.size(); i++)
  {
    if (flight_path[i].simTimeMsec >= elapsed_msec)
    {
      return (int)i;
    }
  }

  // If elapsed time has gone beyond the path, return last segment
  return (int)(flight_path.size() - 1);
}

// MSP channel conversion functions with correct polarity
int convertRollToMSPChannel(gp_scalar gp_command)
{
  // Roll: GP +1.0 = roll right = MSP 2000 (DIRECT mapping)
  gp_scalar clamped = CLAMP_DEF(gp_command, -1.0f, 1.0f);
  return (int)(1500.0f + clamped * 500.0f);
}

int convertPitchToMSPChannel(gp_scalar gp_command)
{
  // Pitch: GP +1.0 = pitch up = MSP 1000 (INVERTED mapping to match CRRCSim)
  gp_scalar clamped = CLAMP_DEF(gp_command, -1.0f, 1.0f);
  return (int)(1500.0f - clamped * 500.0f);
}

int convertThrottleToMSPChannel(gp_scalar gp_command)
{
  // Throttle: GP +1.0 = full throttle = MSP 2000 (DIRECT mapping)
  gp_scalar clamped = CLAMP_DEF(gp_command, -1.0f, 1.0f);
  return (int)(1500.0f + clamped * 500.0f);
}
static void startMspSendTicker()
{
  noInterrupts();
  if (!mspSendTickerRunning)
  {
    mspSendTicker.attach_us(mbed::callback(mspSendTimerHandler), MSP_SEND_INTERVAL_MSEC * 1000);
    mspSendTickerRunning = true;
  }
  interrupts();
}

static void stopMspSendTicker()
{
  bool isrActive = false;
  noInterrupts();
  if (mspSendTickerRunning)
  {
    mspSendTicker.detach();
    mspSendTickerRunning = false;
  }
  if (mspBusLocked)
  {
    isrActive = true;
  }
  else
  {
    mspBusLocked = true; // prevent ISR from entering while we flush
  }
  interrupts();

  if (isrActive)
  {
    // Busy-wait until ISR releases the lock (should be very short)
    while (true)
    {
      noInterrupts();
      bool locked = mspBusLocked;
      if (!locked)
      {
        mspBusLocked = true;
        interrupts();
        break;
      }
      interrupts();
      delayMicroseconds(10);
    }
  }

  // At this point we own the lock exclusively; safe to disable pending flag
  noInterrupts();
  mspSendPending = false;
  mspBusLocked = false;
  interrupts();
}
