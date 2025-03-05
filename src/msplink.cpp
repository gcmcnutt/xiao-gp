#include <main.h>

MSP msp;

bool autoc_enabled = false;
bool status_valid = false;
msp_set_raw_rc_t command_buffer;

void msplinkSetup()
{
  // Initialize MSPLink input serial1 port
  Serial1.begin(115200);
  msp.begin(Serial1, MSP_REPLY_TIMOUT_MSEC);
  logPrint(INFO, "MSPLink Reader Started");

  // set 'valid' values for now
  for (int i = 0; i < MSP_MAX_SUPPORTED_CHANNELS; i++)
  {
    command_buffer.channel[i] = 1500;
  }
}

void mspUpdateState()
{
  static unsigned long lastQueryTime = 0;

  if (millis() - lastQueryTime >= MSP_UPDATE_INTERVAL_MSEC)
  {
    lastQueryTime = millis();

    // get status
    msp_status_t status;
    if (msp.request(MSP_STATUS, &status, sizeof(status)))
    {
      logPrint(DEBUG, "STATUS - Cycle Time: %d, I2C Error Counter: %d, Sensor: %d, Flight Mode Flags: %d, Config Profile Index: %d",
               status.cycleTime, status.i2cErrorCounter, status.sensor, status.flightModeFlags, status.configProfileIndex);

      // TODO interpret
      status_valid = true;
    }
    else
    {
      logPrint(ERROR, "*** Failed to get status");
      status_valid = false;
      return;
    }

    // get RC data
    msp_rc_t rc;
    if (msp.request(MSP_RC, &rc, sizeof(rc)))
    {
      logPrint(DEBUG, "RC_CHANNELS - Ch1: %d, Ch2: %d, Ch3: %d, Ch4: %d, Ch9: %d",
               rc.channelValue[0], rc.channelValue[1], rc.channelValue[2], rc.channelValue[3], rc.channelValue[8]);

      // edge detection for autoc enable/disable
      if (rc.channelValue[8] > 1600 && autoc_enabled == false)
      {
        autoc_enabled = true;
        logPrint(INFO, "Autoc enabled");
        analogWrite(GREEN_PIN, 0);
      }
      else if (rc.channelValue[8] <= 1400 && autoc_enabled == true)
      {
        autoc_enabled = false;
        logPrint(INFO, "Autoc disabled");
        analogWrite(GREEN_PIN, 255);
      }
    }
    else
    {
      logPrint(ERROR, "*** Failed to get RC data");
    }

    // attitude quaternion
    msp_attitude_quaternion_t attitude_quaternion;
    if (msp.request(MSP_ATTITUDE_QUATERNION, &attitude_quaternion, sizeof(attitude_quaternion)))
    {
      logPrint(DEBUG, "ATTITUDE_QUATERNION - q0: %f, q1: %f, q2: %f, q3: %f",
               attitude_quaternion.q[0], attitude_quaternion.q[1], attitude_quaternion.q[2], attitude_quaternion.q[3]);
    }
    else
    {
      logPrint(ERROR, "*** Failed to get attitude quaternion");
    }

    // location
    msp_raw_gps_t raw_gps;
    if (msp.request(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps)))
    {
      logPrint(DEBUG, "RAW_GPS - FixType: %d, Num Satellites: %d, Lat: %d, Lon: %d, Alt: %d, Ground Speed: %d, Ground Course: %d, HDOP: %d",
               raw_gps.fixType, raw_gps.numSat, raw_gps.lat, raw_gps.lon, raw_gps.alt, raw_gps.groundSpeed, raw_gps.groundCourse, raw_gps.hdop);
    }
    else
    {
      logPrint(ERROR, "*** Failed to get raw GPS data");
    }

    // comp_gps
    msp_comp_gps_t comp_gps;
    if (msp.request(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps)))
    {
      logPrint(DEBUG, "COMP_GPS - Distance to Home: %d, Direction to Home: %d, GPS Heartbeat: %d",
               comp_gps.distanceToHome, comp_gps.directionToHome, comp_gps.heartbeat);
    }
    else
    {
      logPrint(ERROR, "*** Failed to get comp GPS data");
    }
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
    if (autoc_enabled && status_valid)
    {
      int c1 = 1500 + 500 * sin(millis() / 1000.0);
      int c2 = 1500 + 500 * cos(millis() / 1000.0);
      int c3 = 1500 + 500 * sin(millis() / 2000.0);

      command_buffer.channel[0] = c1;
      command_buffer.channel[1] = c2;
      command_buffer.channel[2] = c3;
      msp.send(MSP_SET_RAW_RC, &command_buffer, sizeof(command_buffer));
      logPrint(DEBUG, ">>> Sent MSP_SET_RAW_RC: CH1=%d, CH2=%d, CH3=%d", c1, c2, c3);
    }
  }
}
