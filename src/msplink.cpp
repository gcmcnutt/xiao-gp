#include <main.h>
#include <GP/autoc/aircraft_state.h>
#include <embedded_pathgen.h>
#include <GP/autoc/gp_evaluator_embedded.h>
#include <gp_program.h>
#include <vector>

MSP msp;

State state;

// GP Rabbit Path Following System
static EmbeddedLongSequentialPath path_generator;
static std::vector<Path> flight_path;
static AircraftState aircraft_state;
static Path gp_path_segment; // Single GP-compatible path segment for evaluator

// GP Control Timing and State
static unsigned long rabbit_start_time = 0;
static bool rabbit_active = false;
static int current_path_index = 0;

// MSP Control Output Caching
static int cached_roll_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static int cached_pitch_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static int cached_throttle_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static unsigned long lastSendTime = 0;

// Initial waypoint state for relative position calculations
static bool initial_waypoint_cached = false;
static int32_t initial_lat = 0;
static int32_t initial_lon = 0;
static int32_t initial_alt = 0;

// Aircraft state tracking for position/velocity calculation
static unsigned long prev_time = 0;
static Eigen::Vector3d last_valid_position(0.0, 0.0, 0.0);
static bool have_valid_position = false;
static Eigen::Vector3d prev_position(0.0, 0.0, 0.0);
static bool prev_position_valid = false;

// Safety timeout for single GP test run (60 seconds max per run)
#define GP_MAX_SINGLE_RUN_MSEC (60 * 1000)

// Unified GP State logging function
void logGPState()
{
  // Get position/velocity data if available
  String pos_str = "-", vel_str = "-", relvel_str = "-";
  unsigned long time_val = state.asOfMsec;
  
  if (initial_waypoint_cached && !flight_path.empty())
  {
    Eigen::Vector3d pos = aircraft_state.getPosition();
    Eigen::Vector3d vel = aircraft_state.getVelocity();
    double relvel = aircraft_state.getRelVel();
    
    pos_str = String("[") + String(pos[0], 1) + "," + String(pos[1], 1) + "," + String(pos[2], 1) + "]";
    vel_str = String("[") + String(vel[0], 1) + "," + String(vel[1], 1) + "," + String(vel[2], 1) + "]";
    relvel_str = String(relvel, 1);
    
    if (rabbit_active) {
      time_val = state.asOfMsec - rabbit_start_time; // Relative time during test
    } else {
      time_val = 0; // Test not active
    }
  }
  
  // Get attitude data if available using INAV's exact conversion method
  String att_str = "-";
  if (state.attitude_quaternion_valid)
  {
    // Use INAV's quaternion element names: q0=w, q1=x, q2=y, q3=z
    float q0 = state.attitude_quaternion.q[0]; // w
    float q1 = state.attitude_quaternion.q[1]; // x  
    float q2 = state.attitude_quaternion.q[2]; // y
    float q3 = state.attitude_quaternion.q[3]; // z
    
    // Compute rotation matrix exactly like INAV
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
    
    float rMat[3][3];
    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);
    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);
    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
    
    // Compute Euler angles exactly like INAV (in decidegrees, convert to degrees)
    float roll_deg = atan2f(rMat[2][1], rMat[2][2]) * 180.0f / M_PI;
    float pitch_deg = (0.5f * M_PI - acosf(-rMat[2][0])) * 180.0f / M_PI;
    float yaw_deg = -atan2f(rMat[1][0], rMat[0][0]) * 180.0f / M_PI;
    
    // Handle yaw wraparound like INAV
    if (yaw_deg < 0) yaw_deg += 360.0f;
    
    att_str = String("[") + String(roll_deg, 1) + "," + String(pitch_deg, 1) + "," + String(yaw_deg, 1) + "]";
  }
  
  // Get state flags and RC channels
  bool armed = state.status_valid && state.status.flightModeFlags & (1 << MSP_MODE_ARM);
  bool failsafe = state.status_valid && state.status.flightModeFlags & (1 << MSP_MODE_FAILSAFE);
  int rc_ch1 = state.rc_valid ? state.rc.channelValue[0] : 0; // Roll
  int rc_ch2 = state.rc_valid ? state.rc.channelValue[1] : 0; // Pitch
  int rc_ch4 = state.rc_valid ? state.rc.channelValue[3] : 0; // Throttle
  int rc_ch9 = state.rc_valid ? state.rc.channelValue[8] : 0; // Switch channel
  
  // Get quaternion data if available
  String quat_str = "-";
  if (state.attitude_quaternion_valid)
  {
    quat_str = String("[") + String(state.attitude_quaternion.q[0], 3) + "," + String(state.attitude_quaternion.q[1], 3) + 
               "," + String(state.attitude_quaternion.q[2], 3) + "," + String(state.attitude_quaternion.q[3], 3) + "]";
  }
  
  logPrint(INFO, "GP State: pos=%s vel=%s att=%s quat=%s relvel=%s armed=%s fs=%s autoc=%s rc=[%d,%d,%d,%d] time=%lums",
           pos_str.c_str(), vel_str.c_str(), att_str.c_str(), quat_str.c_str(), relvel_str.c_str(),
           armed ? "Y" : "N", failsafe ? "Y" : "N", state.autoc_enabled ? "Y" : "N", 
           rc_ch1, rc_ch2, rc_ch4, rc_ch9, time_val);
}

static void mspUpdateGPControl()
{
  // Check for disarm or failsafe conditions before GP control
  // Only check if we're currently enabled to avoid repeat logging
  if (state.autoc_enabled && rabbit_active)
  {
    bool isArmed = state.status_valid && state.status.flightModeFlags & (1 << MSP_MODE_ARM);
    bool isFailsafe = state.status_valid && state.status.flightModeFlags & (1 << MSP_MODE_FAILSAFE);

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
      state.autoc_enabled = false;
      rabbit_active = false;
      initial_waypoint_cached = false; // Reset position reference for next test
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
    rabbit_active = false;
    return;
  }

  // Find current path segment based on elapsed time since autoc enabled
  current_path_index = getRabbitPathIndex(elapsed_msec);

  // End of path check
  if (current_path_index >= (int)flight_path.size() - 1)
  {
    logPrint(INFO, "GP Control: End of path reached (%.1fs) - stopping rabbit", elapsed_msec / 1000.0);
    rabbit_active = false;
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
    
    double gp_output = generatedGPProgram(pathProvider, aircraft_state, 0.0);

    // Convert GP-controlled aircraft commands to MSP RC values and cache them
    cached_roll_cmd = convertRollToMSPChannel(aircraft_state.getRollCommand());
    cached_pitch_cmd = convertPitchToMSPChannel(aircraft_state.getPitchCommand());
    cached_throttle_cmd = convertThrottleToMSPChannel(aircraft_state.getThrottleCommand());

    logPrint(INFO, "GP Eval: target=[%.1f,%.1f,%.1f] idx=%d cmd=[%d,%d,%d] out=%.3f time=%lums",
             gp_path_segment.start[0], gp_path_segment.start[1], gp_path_segment.start[2],
             current_path_index, cached_roll_cmd, cached_pitch_cmd, cached_throttle_cmd, gp_output, elapsed_msec);
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
    logPrint(ERROR, "*** CRITICAL: Failed to get MSP_STATUS - aborting MSP update cycle");
    return; // Critical error - can't continue without status
  }

  // get RC data
  state.rc_valid = msp.request(MSP_RC, &state.rc, sizeof(state.rc));

  // attitude quaternion
  state.attitude_quaternion_valid = msp.request(MSP_ATTITUDE_QUATERNION, &state.attitude_quaternion, sizeof(state.attitude_quaternion));

  // current position waypoint (waypoint #255 = current estimated position)
  msp_wp_request_t wp_request;
  wp_request.waypointNumber = MSP_WP_CURRENT_POSITION;

  // Send waypoint request and receive waypoint response
  msp.send(MSP_WP, &wp_request, sizeof(wp_request));
  uint16_t recvSize;
  state.waypoint_valid = msp.waitFor(MSP_WP, &state.waypoint, sizeof(state.waypoint), &recvSize);

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

  // then, check the flight mode flags to see if can auto-enable
  if (isArmed && state.rc.channelValue[MSP_ARM_CHANNEL] > MSP_ARMED_THRESHOLD)
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
    // Rising edge: enable autoc and initialize rabbit system
    
    // 1. Cache initial waypoint state for relative position calculations
    if (state.waypoint_valid)
    {
      initial_lat = state.waypoint.lat;
      initial_lon = state.waypoint.lon;
      initial_alt = state.waypoint.alt;
      initial_waypoint_cached = true;

      // 2. Generate longSequential path using embedded pathgen system
      path_generator.generatePath(40.0, 100.0, SIM_INITIAL_ALTITUDE);
      path_generator.copyToVector(flight_path);

      // 3. Start time-based rabbit system
      rabbit_start_time = millis();
      rabbit_active = true;
      current_path_index = 0;

      // Enable autoc and indicate success
      state.autoc_enabled = true;
      analogWrite(GREEN_PIN, 0);
      logPrint(INFO, "GP Control: Switch enabled - cached waypoint lat=%d, lon=%d, alt=%d - starting flight test", initial_lat, initial_lon, initial_alt);
    }
    else
    {
      // Fatal error - cannot start GP control without valid waypoint data
      initial_waypoint_cached = false;
      logPrint(ERROR, "*** FATAL: No valid waypoint data available for GP control - cannot enable autoc");
      // Do not enable autoc_enabled or change LED state
    }
  }
  else if (!new_autoc_enabled && state.autoc_enabled)
  {
    // Falling edge: disable autoc and stop rabbit system
    state.autoc_enabled = false;
    analogWrite(GREEN_PIN, 255);
    if (rabbit_active)
    {
      unsigned long test_run_duration = millis() - rabbit_start_time;
      logPrint(INFO, "GP Control: Switch disabled (%.1fs) - stopping test run", test_run_duration / 1000.0);
    }
    rabbit_active = false;
    initial_waypoint_cached = false; // Reset cached state
  }

  // Update aircraft state on every MSP cycle for continuous position/velocity tracking
  if (initial_waypoint_cached)
  {
    convertMSPStateToAircraftState(aircraft_state);
  }

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

  // Use MSP data that was already collected in mspUpdateState()
  if (!state.attitude_quaternion_valid)
  {
    return;
  }

  // Convert MSP quaternion to Eigen quaternion (adjusted for CRRCsim coordinate system)
  Eigen::Quaterniond orientation(
      state.attitude_quaternion.q[0],  // w (q0)
      state.attitude_quaternion.q[1],  // x (q1)
      state.attitude_quaternion.q[2],  // y (q2)
      state.attitude_quaternion.q[3]   // z (q3)
  );

  // Calculate position in NED coordinates using relative offset from initial waypoint
  Eigen::Vector3d position;
  
  if (state.waypoint_valid && initial_waypoint_cached)
  {
    // Compute differences from armed position in INAV units, then convert to meters
    double lat_diff_deg = (state.waypoint.lat - initial_lat) / 1.0e7;
    double lon_diff_deg = (state.waypoint.lon - initial_lon) / 1.0e7;
    int32_t alt_diff_cm = state.waypoint.alt - initial_alt;

    // Convert to local meters (difference from armed position)
    double north = lat_diff_deg * 111320.0; // degrees to meters
    double east = lon_diff_deg * 111320.0 * cos(initial_lat / 1.0e7 * M_PI / 180.0);
    double down = -alt_diff_cm / 100.0;  // NED: down positive, cm to meters

    // Armed position appears at path origin (0, 0, SIM_INITIAL_ALTITUDE)
    Eigen::Vector3d local_position(north, east, down);
    position = local_position + Eigen::Vector3d(0.0, 0.0, SIM_INITIAL_ALTITUDE);
    last_valid_position = position;
    have_valid_position = true;
  }
  else
  {
    // Coast with previous position if we have one, otherwise use origin
    if (have_valid_position)
    {
      position = last_valid_position;
    }
    else
    {
      position = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
  }

  // Calculate velocity vector from position differentials
  Eigen::Vector3d velocity;

  if (prev_time > 0 && prev_position_valid)
  {
    double dt = (state.asOfMsec - prev_time) / 1000.0; // ms to seconds
    if (dt > 0)
    {
      // Calculate velocity as position differential
      velocity = (position - prev_position) / dt;
    }
    else
    {
      velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
  }
  else
  {
    velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
  }

  // Update previous state for next calculation
  prev_position = position;
  prev_position_valid = true;
  prev_time = state.asOfMsec;

  // Calculate speed magnitude for getRelVel() - mostly vertical velocity
  double speed_magnitude = velocity.norm();

  // Update the AircraftState with current sensor data (all in correct units: meters, m/s)
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
