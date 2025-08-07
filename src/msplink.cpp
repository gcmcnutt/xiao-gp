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
static bool autoc_enabled_prev = false;
static unsigned long rabbit_start_time = 0;
static bool rabbit_active = false;
static int current_path_index = 0;
static Path gp_path_segment; // Single GP-compatible path segment for evaluator
static unsigned long last_gp_eval_time = 0;
static int cached_roll_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static int cached_pitch_cmd = MSP_DEFAULT_CHANNEL_VALUE;  
static int cached_throttle_cmd = MSP_DEFAULT_CHANNEL_VALUE;

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
  if (state.status_valid = msp.request(MSP_STATUS, &state.status, sizeof(state.status)))
  {
    logPrint(DEBUG, "STATUS {CycleTime:%d, FlightModeFlags:0x%x}",
             state.status.cycleTime, state.status.flightModeFlags);
  }
  else
  {
    logPrint(ERROR, "*** Failed to get status");
    return;
  }

  // get RC data
  if (state.rc_valid = msp.request(MSP_RC, &state.rc, sizeof(state.rc)))
  {
    logPrint(DEBUG, "RC_CHANNELS {1:%d, 2:%d, 3:%d, 4:%d, 9:%d}",
             state.rc.channelValue[0], state.rc.channelValue[1], state.rc.channelValue[2], state.rc.channelValue[3], state.rc.channelValue[8]);
  }
  else
  {
    logPrint(ERROR, "*** Failed to get RC data");
  }

  // attitude quaternion
  if (state.attitude_quaternion_valid = msp.request(MSP_ATTITUDE_QUATERNION, &state.attitude_quaternion, sizeof(state.attitude_quaternion)))
  {
    logPrint(DEBUG, "ATTITUDE_QUATERNION {q0:%f, q1:%f, q2:%f, q3:%f}",
             state.attitude_quaternion.q[0], state.attitude_quaternion.q[1], state.attitude_quaternion.q[2], state.attitude_quaternion.q[3]);
  }
  else
  {
    logPrint(ERROR, "*** Failed to get attitude quaternion");
  }

  // location
  if (state.raw_gps_valid = msp.request(MSP_RAW_GPS, &state.raw_gps, sizeof(state.raw_gps)))
  {
    logPrint(DEBUG, "RAW_GPS {FixType:%d, NumSatellites:%d, LatDeg:%f, LonDeg:%f, AltM:%d, GroundSpeed:%d, GroundCourse:%d, HDOP:%d}",
             state.raw_gps.fixType, state.raw_gps.numSat, state.raw_gps.lat / 1.0e7, state.raw_gps.lon / 1.0e7, state.raw_gps.alt, state.raw_gps.groundSpeed, state.raw_gps.groundCourse, state.raw_gps.hdop);
  }
  else
  {
    logPrint(ERROR, "*** Failed to get raw GPS data");
  }

  // comp_gps
  if (state.comp_gps_valid = msp.request(MSP_COMP_GPS, &state.comp_gps, sizeof(state.comp_gps)))
  {
    logPrint(DEBUG, "COMP_GPS {DistM:%d, DirDeg:%d}",
             state.comp_gps.distanceToHome, state.comp_gps.directionToHome);
  }
  else
  {
    logPrint(ERROR, "*** Failed to get comp GPS data");
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

  // then, check the flight mode flags to see if can auto-enable
  if (isArmed && state.rc.channelValue[MSP_ARM_CHANNEL] > MSP_ARMED_THRESHOLD)
  {
    state.autoc_countdown++;
  }
  else
  {
    state.autoc_countdown = 0;
  }

  // then, countdown timer on channel 9 to see if we should auto-enable
  if (state.autoc_countdown > MSP_ARM_CYCLE_COUNT &&
      state.autoc_enabled == false)
  {
    state.autoc_enabled = true;
    logPrint(INFO, "Autoc enabled");
    analogWrite(GREEN_PIN, 0);
  }
  else if (state.autoc_countdown <= MSP_ARM_CYCLE_COUNT &&
           state.autoc_enabled == true)
  {
    state.autoc_enabled = false;
    logPrint(INFO, "Autoc disabled");
    analogWrite(GREEN_PIN, 255);
  }

  // Edge detection for autoc_enabled state
  if (state.autoc_enabled && !autoc_enabled_prev) {
    // Rising edge: initialize rabbit path following system
    initializeRabbitPathSystem();
    logPrint(INFO, "Rabbit path system initialized - starting flight test");
  }
  else if (!state.autoc_enabled && autoc_enabled_prev) {
    // Falling edge: stop rabbit system
    rabbit_active = false;
    logPrint(INFO, "Rabbit path system stopped");
  }
  
  autoc_enabled_prev = state.autoc_enabled;
}

void mspSetControls()
{
  // send interval
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= MSP_SEND_INTERVAL_MSEC)
  {
    lastSendTime = millis();

    // GP rabbit path following control
    if (state.autoc_enabled && rabbit_active && !flight_path.empty())
    {
      unsigned long current_time = millis();
      bool need_new_gp_eval = (current_time - last_gp_eval_time >= MSP_UPDATE_INTERVAL_MSEC);
      
      if (need_new_gp_eval) {
        // GP evaluation every 200ms - calculate new commands
        unsigned long elapsed_msec = current_time - rabbit_start_time;
        double elapsed_sec = elapsed_msec / 1000.0;
        double rabbit_distance = SIM_RABBIT_VELOCITY * elapsed_sec;
        
        // Find current path segment based on rabbit distance
        current_path_index = getRabbitPathIndex(rabbit_distance);
        if (current_path_index < flight_path.size()) {
          gp_path_segment = flight_path[current_path_index];
          
          // Convert MSP state to AircraftState for GP evaluator
          convertMSPStateToAircraftState(aircraft_state);
          
          // Call generated GP program directly
          SinglePathProvider pathProvider(gp_path_segment, aircraft_state.getThisPathIndex());
          double gp_output = generatedGPProgram(pathProvider, aircraft_state, 0.0);
          
          // Convert GP-controlled aircraft commands to MSP RC values and cache them
          cached_roll_cmd = GPEvaluatorEmbedded::convertToMSPChannel(aircraft_state.getRollCommand());
          cached_pitch_cmd = GPEvaluatorEmbedded::convertToMSPChannel(aircraft_state.getPitchCommand());
          cached_throttle_cmd = GPEvaluatorEmbedded::convertToMSPChannel(aircraft_state.getThrottleCommand());
          
          last_gp_eval_time = current_time;
          
          logPrint(INFO, "GP Eval: pos=[%.1f,%.1f,%.1f] target=[%.1f,%.1f,%.1f] dist=%.1f idx=%d cmd=[%d,%d,%d] out=%.3f", 
                   aircraft_state.getPosition()[0], aircraft_state.getPosition()[1], aircraft_state.getPosition()[2],
                   gp_path_segment.start[0], gp_path_segment.start[1], gp_path_segment.start[2],
                   rabbit_distance, current_path_index, cached_roll_cmd, cached_pitch_cmd, cached_throttle_cmd, gp_output);
        }
      }
      
      // Send cached commands every 100ms to prevent INAV timeout
      state.command_buffer.channel[0] = cached_roll_cmd;    // Roll
      state.command_buffer.channel[1] = cached_pitch_cmd;   // Pitch  
      state.command_buffer.channel[2] = cached_throttle_cmd; // Throttle
      msp.send(MSP_SET_RAW_RC, &state.command_buffer, sizeof(state.command_buffer));
      
      if (!need_new_gp_eval) {
        logPrint(DEBUG, "GP Send: cached cmd=[%d,%d,%d]", cached_roll_cmd, cached_pitch_cmd, cached_throttle_cmd);
      }
    }
  }
}

// Initialize the rabbit path following system when autoc_enabled goes true
void initializeRabbitPathSystem() {
  // 1. Set current craft state to origin (arm point becomes origin)
  if (state.raw_gps_valid && state.attitude_quaternion_valid) {
    logPrint(INFO, "Origin set to current GPS position (arm point)");
  }
  
  // 2. Generate longSequential path using embedded pathgen system
  path_generator.generatePath(40.0, 100.0, SIM_INITIAL_ALTITUDE);
  path_generator.copyToVector(flight_path);
  
  logPrint(INFO, "Generated GP path with %d segments, length %.1f m", 
           flight_path.size(), path_generator.getTotalPathLength());
  
  // 3. Start time-based rabbit system
  rabbit_start_time = millis();
  rabbit_active = true;
  current_path_index = 0;
  
  // 4. Initialize aircraft state from current MSP data
  convertMSPStateToAircraftState(aircraft_state);
  
  logPrint(INFO, "Rabbit started at velocity %.1f m/s", SIM_RABBIT_VELOCITY);
}

// Convert MSP state data to AircraftState for GP evaluator
void convertMSPStateToAircraftState(AircraftState& aircraftState) {
  // Use MSP data that was already collected in mspUpdateState()
  if (!state.attitude_quaternion_valid || !state.raw_gps_valid) {
    return;
  }
  
  // Convert MSP quaternion to Eigen quaternion
  Eigen::Quaterniond orientation(
    state.attitude_quaternion.q[0],  // w
    state.attitude_quaternion.q[1],  // x
    state.attitude_quaternion.q[2],  // y
    state.attitude_quaternion.q[3]   // z
  );
  
  // Convert GPS position to NED coordinates relative to origin (arm point)
  Eigen::Vector3d position(
    0.0,  // North - use relative positioning from arm point
    0.0,  // East
    SIM_INITIAL_ALTITUDE  // Down (negative altitude)
  );
  
  // Estimate velocity from GPS ground speed and course
  Eigen::Vector3d velocity(
    state.raw_gps.groundSpeed * cos(state.raw_gps.groundCourse * M_PI / 180.0) / 100.0,
    state.raw_gps.groundSpeed * sin(state.raw_gps.groundCourse * M_PI / 180.0) / 100.0,
    0.0
  );
  
  // Update the AircraftState with current sensor data
  aircraftState.setPosition(position);
  aircraftState.setOrientation(orientation);
  aircraftState.setVelocity(velocity);
  aircraftState.setSimTimeMsec(state.asOfMsec);  // Use MSP timestamp
  aircraftState.setThisPathIndex(current_path_index);
}

// Find path index based on rabbit distance along path
int getRabbitPathIndex(double rabbit_distance) {
  if (flight_path.empty()) return 0;
  
  // Find the path segment closest to the rabbit distance
  for (size_t i = 0; i < flight_path.size(); i++) {
    if (flight_path[i].distanceFromStart >= rabbit_distance) {
      return (int)i;
    }
  }
  
  // If rabbit has gone beyond the path, return last segment
  return (int)(flight_path.size() - 1);
}

// convertToMSPChannel now provided by GPEvaluatorEmbedded class
