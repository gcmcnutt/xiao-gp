#include <main.h>

MSP msp;

State state;

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
}

void mspSetControls()
{
  // send interval
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= MSP_SEND_INTERVAL_MSEC)
  {
    lastSendTime = millis();

    // synthesize some output values for pitch/roll
    if (state.autoc_enabled)
    {
      int c1 = 1500 + 500 * sin(millis() / 1000.0);
      int c2 = 1500 + 500 * cos(millis() / 1000.0);
      int c3 = 1000 + (millis() % 2000) / 2;

      state.command_buffer.channel[0] = c1;
      state.command_buffer.channel[1] = c2;
      state.command_buffer.channel[2] = c3;
      msp.send(MSP_SET_RAW_RC, &state.command_buffer, sizeof(state.command_buffer));
      logPrint(DEBUG, ">>> Sent MSP_SET_RAW_RC [1:%d, 2:%d, 3:%d]", c1, c2, c3);

      
    }
  }
}
