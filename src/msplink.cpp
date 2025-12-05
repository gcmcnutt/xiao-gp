#include <main.h>
#include <GP/autoc/aircraft_state.h>
#include <embedded_pathgen.h>
#include <GP/autoc/gp_evaluator_embedded.h>
#include <gp_program.h>
#include <mbed.h>
#include <vector>
#include <cmath>
#include <algorithm>

MSP msp;

State state;

// GP Rabbit Path Following System
static EmbeddedLongSequentialPath path_generator;
static std::vector<Path> flight_path;
static AircraftState aircraft_state;
static Path gp_path_segment; // Single GP-compatible path segment for evaluator

// Path anchoring
static Eigen::Vector3d path_origin_offset(0.0, 0.0, 0.0);
static bool path_origin_set = false;

// GP Control Timing and State
static unsigned long rabbit_start_time = 0;
static volatile bool rabbit_active = false;
static int current_path_index = 0;
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
static Eigen::Vector3d last_valid_position(0.0, 0.0, 0.0);
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
  last_valid_position = Eigen::Vector3d(0.0, 0.0, 0.0);
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
  path_origin_set = false;
  path_origin_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
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

static Eigen::Vector3d neuVectorToNedMeters(const int32_t vec_cm[3])
{
  double north = static_cast<double>(vec_cm[0]) / 100.0;
  double east = static_cast<double>(vec_cm[1]) / 100.0;
  double down = -static_cast<double>(vec_cm[2]) / 100.0;
  return Eigen::Vector3d(north, east, down);
}

static Eigen::Quaterniond neuQuaternionToNed(const float q[4])
{
  // INAV reports attitude as a unit quaternion in the NED earth frame (North-East-Down).
  // No additional axis swapping is required—normalize and pass it through.
  Eigen::Quaterniond attitude(q[0], q[1], q[2], q[3]);
  if (attitude.norm() == 0.0)
  {
    return Eigen::Quaterniond::Identity();
  }
  attitude.normalize();
  return attitude;
}

// Unified GP State logging function
void logGPState()
{
  // Get position/velocity data if available
  String pos_str = "-", vel_str = "-", relvel_str = "-";
  unsigned long time_val = state.asOfMsec;

  // Always show velocity and position data since we now have continuous tracking
  Eigen::Vector3d vel = aircraft_state.getVelocity();
  Eigen::Vector3d pos = aircraft_state.getPosition();
  double relvel = aircraft_state.getRelVel();

  vel_str = String("[") + String(vel[0], 1) + "," + String(vel[1], 1) + "," + String(vel[2], 1) + "]";
  pos_str = String("[") + String(pos[0], 1) + "," + String(pos[1], 1) + "," + String(pos[2], 1) + "]";
  relvel_str = String(relvel, 1);

  // Time data only available when test is active
  if (!flight_path.empty())
  {
    if (rabbit_active)
    {
      if (state.asOfMsec >= rabbit_start_time)
      {
        time_val = state.asOfMsec - rabbit_start_time; // Relative time during test
      }
      else
      {
        time_val = 0;
      }
    }
    else
    {
      time_val = 0; // Test not active
    }
  }
  
  // Get attitude data if available using INAV's quaternion convention (matches Configurator)
  String att_str = "-";
  String quat_str = "-";
  if (state.local_state_valid)
  {
    double q0 = state.local_state.q[0];
    double q1 = state.local_state.q[1];
    double q2 = state.local_state.q[2];
    double q3 = state.local_state.q[3];

    double q1q1 = q1 * q1;
    double q2q2 = q2 * q2;
    double q3q3 = q3 * q3;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q3 = q2 * q3;

    double rMat[3][3];
    rMat[0][0] = 1.0 - 2.0 * q2q2 - 2.0 * q3q3;
    rMat[0][1] = 2.0 * (q1q2 - q0q3);
    rMat[0][2] = 2.0 * (q1q3 + q0q2);
    rMat[1][0] = 2.0 * (q1q2 + q0q3);
    rMat[1][1] = 1.0 - 2.0 * q1q1 - 2.0 * q3q3;
    rMat[1][2] = 2.0 * (q2q3 - q0q1);
    rMat[2][0] = 2.0 * (q1q3 - q0q2);
    rMat[2][1] = 2.0 * (q2q3 + q0q1);
    rMat[2][2] = 1.0 - 2.0 * q1q1 - 2.0 * q2q2;

    double roll_deg = atan2(rMat[2][1], rMat[2][2]) * 180.0 / M_PI;
    double pitch_deg = (0.5 * M_PI - acos(-rMat[2][0])) * 180.0 / M_PI;
    double yaw_deg = -atan2(rMat[1][0], rMat[0][0]) * 180.0 / M_PI;

    if (yaw_deg < 0) yaw_deg += 360.0;

    att_str = String("[") + String(roll_deg, 1) + "," + String(pitch_deg, 1) + "," + String(yaw_deg, 1) + "]";
    quat_str = String("[") + String(q0, 3) + "," + String(q1, 3) + "," + String(q2, 3) + "," + String(q3, 3) + "]";
  }
  else
  {
    Eigen::Quaterniond orientation = aircraft_state.getOrientation();
    if (orientation.norm() > 0.0)
    {
      orientation.normalize();
      Eigen::Vector3d euler = orientation.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
      double yaw_deg = euler[0] * 180.0 / M_PI;
      double pitch_deg = euler[1] * 180.0 / M_PI;
      double roll_deg = euler[2] * 180.0 / M_PI;

      if (yaw_deg < 0) yaw_deg += 360.0;

      att_str = String("[") + String(roll_deg, 1) + "," + String(pitch_deg, 1) + "," + String(yaw_deg, 1) + "]";
      quat_str = String("[") + String(orientation.w(), 3) + "," + String(orientation.x(), 3) + ","
                 + String(orientation.y(), 3) + "," + String(orientation.z(), 3) + "]";
    }
  }
  
  // Get state flags
  bool armed = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_ARM);
  bool failsafe = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_FAILSAFE);

  bool hasServoActivation = state.rc_valid && state.rc.channelValue[MSP_ARM_CHANNEL] > MSP_ARMED_THRESHOLD;

  logPrint(INFO, "GP State: pos=%s vel=%s att=%s quat=%s relvel=%s armed=%s fs=%s servo=%s autoc=%s rabbit=%s",
           pos_str.c_str(), vel_str.c_str(), att_str.c_str(), quat_str.c_str(), relvel_str.c_str(),
           armed ? "Y" : "N", failsafe ? "Y" : "N", hasServoActivation ? "Y" : "N", state.autoc_enabled ? "Y" : "N",
           rabbit_active ? "Y" : "N");
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
        logPrint(INFO, "GP Control: INAV failsafe activated (%.1fs) - disabling autoc", test_run_duration / 1000.0);
      }
      else
      {
        logPrint(INFO, "GP Control: Aircraft disarmed (%.1fs) - disabling autoc", test_run_duration / 1000.0);
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
    logPrint(INFO, "GP Control: Test run timeout (%.1fs) - stopping rabbit", elapsed_msec / 1000.0);
    stopAutoc("timeout", true);
    return;
  }

  // Find current path segment based on elapsed time since autoc enabled
  current_path_index = getRabbitPathIndex(elapsed_msec);

  // End of path check
  if (current_path_index >= (int)flight_path.size() - 1)
  {
    logPrint(INFO, "GP Control: End of path reached (%.1fs) - stopping rabbit", elapsed_msec / 1000.0);
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
    
    // DEBUG: Manual GETDTHETA and GETDPHI evaluation with coordinate details
    double debug_distance = (gp_path_segment.start - aircraft_state.getPosition()).norm();

    // Calculate craft-to-target vector in world frame
    Eigen::Vector3d craftToTarget = gp_path_segment.start - aircraft_state.getPosition();
    // Transform to body frame
    Eigen::Vector3d target_local = aircraft_state.getOrientation().inverse() * craftToTarget;

    double debug_getdtheta = evaluateGPOperator(GETDTHETA, pathProvider, aircraft_state, nullptr, 0, 0.0);
    double debug_getdphi = evaluateGPOperator(GETDPHI, pathProvider, aircraft_state, nullptr, 0, 0.0);
    double debug_getalpha = evaluateGPOperator(GETALPHA, pathProvider, aircraft_state, nullptr, 0, 0.0);
    double debug_getdtarget = evaluateGPOperator(GETDTARGET, pathProvider, aircraft_state, nullptr, 0, 0.0);
    double debug_getbeta = evaluateGPOperator(GETBETA, pathProvider, aircraft_state, nullptr, 0, 0.0);
    double debug_getdhome = evaluateGPOperator(GETDHOME, pathProvider, aircraft_state, nullptr, 0, 0.0);

    // Calculate body-frame velocity for detailed logging
    Eigen::Vector3d velocity_body = aircraft_state.getOrientation().inverse() * aircraft_state.getVelocity();

    // Get raw quaternion
    Eigen::Quaterniond q = aircraft_state.getOrientation();

    logPrint(INFO, "DEBUG GP: world_vec=[%.1f,%.1f,%.1f] body_vec=[%.1f,%.1f,%.1f] theta=%.3f phi=%.3f alpha=%.3f dtarget=%.3f",
             craftToTarget.x(), craftToTarget.y(), craftToTarget.z(),
             target_local.x(), target_local.y(), target_local.z(),
             debug_getdtheta, debug_getdphi, debug_getalpha, debug_getdtarget);

    // NEW: Detailed GP sensor logging for frame convention verification
    logPrint(INFO, "GP SENSORS: quat=[%.4f,%.4f,%.4f,%.4f] vbody=[%.2f,%.2f,%.2f] alpha=%.2f beta=%.2f dtheta=%.2f dphi=%.2f dhome=%.2f",
             q.w(), q.x(), q.y(), q.z(),
             velocity_body.x(), velocity_body.y(), velocity_body.z(),
             debug_getalpha * 180.0 / M_PI,      // Convert radians to degrees
             debug_getbeta * 180.0 / M_PI,
             debug_getdtheta * 180.0 / M_PI,
             debug_getdphi * 180.0 / M_PI,
             debug_getdhome);

    // DEBUG: Check if path is advancing and distance increasing
    logPrint(INFO, "DEBUG RABBIT: idx=%d, elapsed=%lums, target_y=%.1f, craft_y=%.1f, dist=%.1f",
             current_path_index, elapsed_msec, gp_path_segment.start.y(),
             aircraft_state.getPosition().y(), debug_distance);
    
    generatedGPProgram(pathProvider, aircraft_state, 0.0);

    // Convert GP-controlled aircraft commands to MSP RC values and cache them
    int roll_cmd = convertRollToMSPChannel(aircraft_state.getRollCommand());
    int pitch_cmd = convertPitchToMSPChannel(aircraft_state.getPitchCommand());
    int throttle_cmd = convertThrottleToMSPChannel(aircraft_state.getThrottleCommand());
    updateCachedCommands(roll_cmd, pitch_cmd, throttle_cmd, eval_start_us);

    logPrint(INFO, "GP Eval: target=[%.1f,%.1f,%.1f] idx=%d setRcData=[%d,%d,%d]",
             gp_path_segment.start[0], gp_path_segment.start[1], gp_path_segment.start[2],
             current_path_index, roll_cmd, pitch_cmd, throttle_cmd);
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
      path_origin_offset = neuVectorToNedMeters(state.local_state.pos);
      path_origin_set = true;

      path_generator.generatePath(40.0, 100.0, 0.0);
      path_generator.copyToVector(flight_path);
      if (path_origin_set)
      {
        for (auto &segment : flight_path)
        {
          segment.start += path_origin_offset;
        }
      }

      rabbit_start_time = millis();
      rabbit_active = true;
      resetMspSendStats();
      startMspSendTicker();
      current_path_index = 0;

      state.autoc_enabled = true;
      servo_reset_required = false;
      analogWrite(GREEN_PIN, 0);
      logPrint(INFO, "GP Control: Switch enabled - path origin NED=[%.2f, %.2f, %.2f] - starting flight test",
               path_origin_offset.x(), path_origin_offset.y(), path_origin_offset.z());
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
      logPrint(INFO, "GP Control: Switch disabled (%.1fs) - stopping test run", test_run_duration / 1000.0);
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

  // Update GP control and cache commands when enabled
  mspUpdateGPControl();

  // Always log unified GP State with available data or "-" for missing
  logGPState();
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

  double intervalSumMs = 0.0;
  double intervalMinMs = 1e9;
  double intervalMaxMs = 0.0;
  uint32_t intervalCount = 0;
  uint32_t lateIntervals = 0;
  const uint32_t desiredIntervalUs = MSP_SEND_INTERVAL_MSEC * 1000UL;
  const uint32_t lateThresholdUs = desiredIntervalUs + 20000UL; // allow 20ms slack (70ms total)

  double latencyStartSumMs = 0.0;
  double latencyEndSumMs = 0.0;
  double latencyStartMinMs = 1e9;
  double latencyStartMaxMs = 0.0;
  double latencyEndMinMs = 1e9;
  double latencyEndMaxMs = 0.0;
  uint32_t latencySamples = 0;

  uint32_t prevSend = 0;

  for (uint16_t i = 0; i < count; ++i)
  {
    uint16_t idx = (readIdx + i) % MSP_SEND_LOG_CAPACITY;
    const MspSendLogEntry &entry = mspSendLog[idx];

    if (i > 0)
    {
      uint32_t delta = entry.sendTimeUs - prevSend;
      double deltaMs = delta / 1000.0;
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
      double startLatencyMs = (entry.sendTimeUs - entry.evalStartUs) / 1000.0;
      double endLatencyMs = (entry.sendTimeUs - entry.evalEndUs) / 1000.0;
      latencyStartSumMs += startLatencyMs;
      latencyEndSumMs += endLatencyMs;
      latencyStartMinMs = std::min(latencyStartMinMs, startLatencyMs);
      latencyStartMaxMs = std::max(latencyStartMaxMs, startLatencyMs);
      latencyEndMinMs = std::min(latencyEndMinMs, endLatencyMs);
      latencyEndMaxMs = std::max(latencyEndMaxMs, endLatencyMs);
      latencySamples++;
    }
  }

  double intervalAvgMs = intervalCount ? intervalSumMs / intervalCount : 0.0;
  if (intervalCount == 0)
  {
    intervalMinMs = 0.0;
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
    double latencyStartAvgMs = latencyStartSumMs / latencySamples;
    double latencyEndAvgMs = latencyEndSumMs / latencySamples;
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

  Eigen::Vector3d position = have_valid_position ? last_valid_position : Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = aircraftState.getOrientation();

  if (state.local_state_valid)
  {
    position = neuVectorToNedMeters(state.local_state.pos);
    velocity = neuVectorToNedMeters(state.local_state.vel);
    orientation = neuQuaternionToNed(state.local_state.q);

    last_valid_position = position;
    have_valid_position = true;
  }
  // If no new quaternion data, retain previous orientation from aircraft state

  double speed_magnitude = velocity.norm();

  aircraftState.setPosition(position);
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
int convertRollToMSPChannel(double gp_command)
{
  // Roll: GP +1.0 = roll right = MSP 2000 (DIRECT mapping)
  double clamped = CLAMP_DEF(gp_command, -1.0, 1.0);
  return (int)(1500.0 + clamped * 500.0);
}

int convertPitchToMSPChannel(double gp_command)
{
  // Pitch: GP +1.0 = pitch up = MSP 1000 (INVERTED mapping to match CRRCSim)
  double clamped = CLAMP_DEF(gp_command, -1.0, 1.0);
  return (int)(1500.0 - clamped * 500.0);
}

int convertThrottleToMSPChannel(double gp_command)
{
  // Throttle: GP +1.0 = full throttle = MSP 2000 (DIRECT mapping)
  double clamped = CLAMP_DEF(gp_command, -1.0, 1.0);
  return (int)(1500.0 + clamped * 500.0);
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
