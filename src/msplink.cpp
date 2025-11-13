#include <main.h>
#include <GP/autoc/aircraft_state.h>
#include <embedded_pathgen.h>
#include <GP/autoc/gp_evaluator_embedded.h>
#include <gp_program.h>
#include <vector>
#include <cmath>

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
static bool rabbit_active = false;
static int current_path_index = 0;
static bool servo_reset_required = false;

// MSP Control Output Caching
static int cached_roll_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static int cached_pitch_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static int cached_throttle_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static unsigned long lastSendTime = 0;

// Aircraft state tracking for position/velocity calculation
static Eigen::Vector3d last_valid_position(0.0, 0.0, 0.0);
static bool have_valid_position = false;
static bool was_system_armed = false;

// Safety timeout for single GP test run (60 seconds max per run)
#define GP_MAX_SINGLE_RUN_MSEC (60 * 1000)

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

  if (wasAutoc || wasRabbit || (requireServoReset && !latchBefore))
  {
    logPrint(INFO, "GP Control: Autoc disabled (%s) - pilot has control", reason);
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
  Eigen::Quaterniond q_neu(q[0], q[1], q[2], q[3]);
  if (q_neu.norm() == 0.0) {
    return Eigen::Quaterniond::Identity();
  }
  q_neu.normalize();
  
  // NEU→NED: 180° rotation around X-axis (North axis)
  Eigen::Quaterniond q_neu_to_ned(0.0, 1.0, 0.0, 0.0);
  
  // Transform: Earth(NED)→Body = (NEU→NED) * Earth(NEU)→Body
  Eigen::Quaterniond q_ned = q_neu_to_ned * q_neu;
  q_ned.normalize();
  
  return q_ned;
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

  logPrint(INFO, "GP State: pos=%s vel=%s att=%s quat=%s relvel=%s armed=%s fs=%s servo=%s autoc=%s rabbit=%s time=%lums",
           pos_str.c_str(), vel_str.c_str(), att_str.c_str(), quat_str.c_str(), relvel_str.c_str(),
           armed ? "Y" : "N", failsafe ? "Y" : "N", hasServoActivation ? "Y" : "N", state.autoc_enabled ? "Y" : "N",
           rabbit_active ? "Y" : "N",
           time_val);
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
    
    // DEBUG: Manual GETDTHETA and GETDPHI evaluation with coordinate details
    double debug_args[1] = {0.0}; // offset arg = 0
    double debug_distance = (gp_path_segment.start - aircraft_state.getPosition()).norm();
    
    // Calculate craft-to-target vector in world frame
    Eigen::Vector3d craftToTarget = gp_path_segment.start - aircraft_state.getPosition();
    // Transform to body frame
    Eigen::Vector3d target_local = aircraft_state.getOrientation().inverse() * craftToTarget;
    
    double debug_getdtheta = evaluateGPOperator(10, pathProvider, aircraft_state, debug_args, 1, 0.0);  // GETDTHETA opcode = 10
    double debug_getdphi = evaluateGPOperator(9, pathProvider, aircraft_state, debug_args, 1, 0.0);     // GETDPHI opcode = 9
    double debug_getalpha = evaluateGPOperator(18, pathProvider, aircraft_state, debug_args, 1, 0.0);   // GETALPHA opcode = 18
    double debug_getdtarget = evaluateGPOperator(11, pathProvider, aircraft_state, debug_args, 1, 0.0); // GETDTARGET opcode = 11
    
    logPrint(DEBUG, "DEBUG GP: world_vec=[%.1f,%.1f,%.1f] body_vec=[%.1f,%.1f,%.1f] theta=%.3f phi=%.3f alpha=%.3f dtarget=%.3f", 
             craftToTarget.x(), craftToTarget.y(), craftToTarget.z(),
             target_local.x(), target_local.y(), target_local.z(),
             debug_getdtheta, debug_getdphi, debug_getalpha, debug_getdtarget);
    
    // DEBUG: Check if path is advancing and distance increasing
    logPrint(DEBUG, "DEBUG RABBIT: idx=%d, elapsed=%lums, target_y=%.1f, craft_y=%.1f, dist=%.1f", 
             current_path_index, elapsed_msec, gp_path_segment.start.y(), 
             aircraft_state.getPosition().y(), debug_distance);
    
    generatedGPProgram(pathProvider, aircraft_state, 0.0);

    // Convert GP-controlled aircraft commands to MSP RC values and cache them
    cached_roll_cmd = convertRollToMSPChannel(aircraft_state.getRollCommand());
    cached_pitch_cmd = convertPitchToMSPChannel(aircraft_state.getPitchCommand());
    cached_throttle_cmd = convertThrottleToMSPChannel(aircraft_state.getThrottleCommand());

    logPrint(INFO, "GP Eval: target=[%.1f,%.1f,%.1f] idx=%d setRcData=[%d,%d,%d] time=%lums",
             gp_path_segment.start[0], gp_path_segment.start[1], gp_path_segment.start[2],
             current_path_index, cached_roll_cmd, cached_pitch_cmd, cached_throttle_cmd, elapsed_msec);
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
}

void mspUpdateState()
{
  state.resetState();
  state.setAsOfMsec(millis());

  // get status
  state.status_valid = msp.request(MSP_STATUS, &state.status, sizeof(state.status));
  if (!state.status_valid)
  {
    stopAutoc("MSP status failure", true);
    logPrint(ERROR, "*** CRITICAL: Failed to get MSP_STATUS - aborting MSP update cycle");
    return; // Critical error - can't continue without status
  }

  // Local state (position / velocity / quaternion)
  state.local_state_valid = msp.request(MSP2_INAV_LOCAL_STATE, &state.local_state, sizeof(state.local_state));
  // Local state already provides quaternion data; if unavailable, orientation will be held from previous sample
  if (!state.local_state_valid && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc("MSP local state failure", true);
  }

  // RC channels
  state.rc_valid = msp.request(MSP_RC, &state.rc, sizeof(state.rc));
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
  // Only send GP commands when rabbit is active - otherwise let user have control
  if (rabbit_active && millis() - lastSendTime >= MSP_SEND_INTERVAL_MSEC)
  {
    lastSendTime = millis();

    // Send cached GP commands
    state.command_buffer.channel[0] = cached_roll_cmd;     // Roll
    state.command_buffer.channel[1] = cached_pitch_cmd;    // Pitch
    state.command_buffer.channel[2] = cached_throttle_cmd; // Throttle
    msp.send(MSP_SET_RAW_RC, &state.command_buffer, sizeof(state.command_buffer));
  }
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
