#include <main.h>

MSP msp;

bool autoc_enabled = false;
msp_set_raw_rc_t command_buffer;

void msplinkSetup()
{
  // Initialize MSPLink input serial1 port
  Serial1.begin(115200);
  msp.begin(Serial1, 100);
  Serial.println("MSPLink Reader Started");

  // set 'valid' values for now
  for (int i = 0; i < MSP_MAX_SUPPORTED_CHANNELS; i++)
  {
    command_buffer.channel[i] = 1500;
  }
}

void mspQueryLoop()
{
  static unsigned long lastQueryTime = 0;

  if (millis() - lastQueryTime >= MSP_UPDATE_INTERVAL_MSEC)
  {
    lastQueryTime = millis();

    // get status
    msp_status_t status;
    if (msp.request(MSP_STATUS, &status, sizeof(status)))
    {
      Serial.print("STATUS - Cycle Time: ");
      Serial.print(status.cycleTime);
      Serial.print(", I2C Error Counter: ");
      Serial.print(status.i2cErrorCounter);
      Serial.print(", Sensor: ");
      Serial.print(status.sensor);
      Serial.print(", Flight Mode Flags: ");
      Serial.print(status.flightModeFlags);
      Serial.print(", Config Profile Index: ");
      Serial.println(status.configProfileIndex);
    }
    else
    {
      Serial.println("*** Failed to get status");
    }

    // get RC data
    msp_rc_t rc;
    if (msp.request(MSP_RC, &rc, sizeof(rc)))
    {
      Serial.print("  RC_CHANNELS - Ch1: ");
      Serial.print(rc.channelValue[0]);
      Serial.print(", Ch2: ");
      Serial.print(rc.channelValue[1]);
      Serial.print(", Ch3: ");
      Serial.print(rc.channelValue[2]);
      Serial.print(", Ch4: ");
      Serial.print(rc.channelValue[3]);
      Serial.print(", Ch9: ");
      Serial.println(rc.channelValue[8]);

      // edge detection for autoc enable/disable
      if (rc.channelValue[8] > 1600 && autoc_enabled == false)
      {
        autoc_enabled = true;
        Serial.println("Autoc enabled");
        analogWrite(GREEN_PIN, 0);
      }
      else if (rc.channelValue[8] <= 1400 && autoc_enabled == true)
      {
        autoc_enabled = false;
        Serial.println("Autoc disabled");
        analogWrite(GREEN_PIN, 255);
      }
    }
    else
    {
      Serial.println("*** Failed to get RC data");
    }

    // attitude quaternion
    msp_attitude_quaternion_t attitude_quaternion;
    if (msp.request(MSP_ATTITUDE_QUATERNION, &attitude_quaternion, sizeof(attitude_quaternion)))
    {
      Serial.print("  ATTITUDE_QUATERNION - q0: ");
      Serial.print(attitude_quaternion.q[0]);
      Serial.print(", q1: ");
      Serial.print(attitude_quaternion.q[1]);
      Serial.print(", q2: ");
      Serial.print(attitude_quaternion.q[2]);
      Serial.print(", q3: ");
      Serial.println(attitude_quaternion.q[3]);
    }
    else
    {
      Serial.println("*** Failed to get attitude quaternion");
    }

    // location
    msp_raw_gps_t raw_gps;
    if (msp.request(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps)))
    {
      Serial.print("  RAW_GPS - FixType: ");
      Serial.print(raw_gps.fixType);
      Serial.print(", Num Satellites: ");
      Serial.print(raw_gps.numSat);
      Serial.print(", Lat: ");
      Serial.print(raw_gps.lat);
      Serial.print(", Lon: ");
      Serial.print(raw_gps.lon);
      Serial.print(", Alt: ");
      Serial.print(raw_gps.alt);
      Serial.print(", Ground Speed: ");
      Serial.print(raw_gps.groundSpeed);
      Serial.print(", Ground Course: ");
      Serial.print(raw_gps.groundCourse);
      Serial.print(", HDOP: ");
      Serial.println(raw_gps.hdop);
    }
    else
    {
      Serial.println("*** Failed to get raw GPS data");
    }

    // comp_gps
    msp_comp_gps_t comp_gps;
    if (msp.request(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps)))
    {
      Serial.print("  COMP_GPS - Distance to Home: ");
      Serial.print(comp_gps.distanceToHome);
      Serial.print(", Direction to Home: ");
      Serial.print(comp_gps.directionToHome);
      Serial.print(", GPS Heartbeat: ");
      Serial.println(comp_gps.heartbeat);
    }
    else
    {
      Serial.println("*** Failed to get comp GPS data");
    }
  }
}

void mspSendLoop()
{
  // send interval
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime >= MSP_SEND_INTERVAL_MSEC)
  {
    lastSendTime = millis();

    // synthesize some output values for pitch/roll
    if (autoc_enabled)
    {
      double c1 = 1500 + 500 * sin(millis() / 1000.0);
      double c2 = 1500 + 500 * cos(millis() / 1000.0);
      double c3 = 1500 + 500 * sin(millis() / 2000.0);

      command_buffer.channel[0] = c1;
      command_buffer.channel[1] = c2;
      command_buffer.channel[2] = c3;
      msp.send(MSP_SET_RAW_RC, &command_buffer, sizeof(command_buffer));
      Serial.println(">>> Sent MSP_SET_RAW_RC: CH1=" + String(c1) + ", CH2=" + String(c2) + ", CH3=" + String(c3));
    }
  }
}

void msplinkLoop()
{
  mspQueryLoop();
  mspSendLoop();
}
