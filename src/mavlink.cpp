#include <main.h>

// Define the input serial port (e.g., UART connected to MAVLink source)
HardwareSerial &inputSerial = Serial1;

void request_telemetry()
{
  // mavlink_message_t msg;
  // mavlink_command_long_t cmd;
  // cmd.target_system = 1;
  // cmd.target_component = 250;
  // cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;

  // cmd.param1 = MAVLINK_MSG_ID_RC_CHANNELS;
  // cmd.param2 = 200000; // 5 Hz
  // mavlink_msg_command_long_encode(100, 1, &msg, &cmd);
  // uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // inputSerial.write(buf, len);
  // Serial.println("Requested RC_CHANNELS");
}

void mavlinkSetup()
{
  // Initialize MAVLink input serial port
  inputSerial.begin(115200); // Match the baud rate of your MAVLink source
  Serial.println("MAVLink Reader Started");

  delay(1000); // Wait for the MAVLink source to boot
  request_telemetry();
}

void processMessage(mavlink_message_t *msg)
{
  // // Handle the message based on its ID
  // Serial.print("Received message ID: ");
  // Serial.print(msg->msgid);
  // Serial.print(" from System ID: ");
  // Serial.print(msg->sysid);
  // Serial.print(", Component ID: ");
  // Serial.println(msg->compid);

  // Example: Handle specific message types
  switch (msg->msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
  {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);
    Serial.print("  HEARTBEAT from System: ");
    Serial.print(msg->sysid);
    Serial.print(", Component: ");
    Serial.print(msg->compid);
    Serial.print(", Type: ");
    Serial.print(heartbeat.type);
    Serial.print(", Autopilot: ");
    Serial.println(heartbeat.autopilot);
    break;
  }
  case MAVLINK_MSG_ID_GPS_RAW_INT:
  {
    mavlink_gps_raw_int_t gps;
    mavlink_msg_gps_raw_int_decode(msg, &gps);
    Serial.print("  GPS - Fix Type: ");
    Serial.print(gps.fix_type);
    Serial.print(", Lat: ");
    Serial.print(gps.lat / 1e7); // Convert to degrees
    Serial.print(", Lon: ");
    Serial.println(gps.lon / 1e7);
    break;
  }
  case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
  {
    mavlink_global_position_int_t global_pos;
    mavlink_msg_global_position_int_decode(msg, &global_pos);
    Serial.print("  GLOBAL_POSITION_INT - Lat: ");
    Serial.print(global_pos.lat / 1e7); // Convert to degrees
    Serial.print(", Lon: ");
    Serial.print(global_pos.lon / 1e7);
    Serial.print(", Alt: ");
    Serial.println(global_pos.alt / 1000); // Convert to meters
    break;
  }
  case MAVLINK_MSG_ID_ATTITUDE:
  {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(msg, &attitude);
    Serial.print("  ATTITUDE - Roll: ");
    Serial.print(attitude.roll);
    Serial.print(", Pitch: ");
    Serial.print(attitude.pitch);
    Serial.print(", Yaw: ");
    Serial.println(attitude.yaw);
    break;
  }

  // TODO this starts showing up in inav 8.0
  case MAVLINK_MSG_ID_RC_CHANNELS:
  {
    mavlink_rc_channels_t rc_channels;
    mavlink_msg_rc_channels_decode(msg, &rc_channels);
    Serial.print("  RC_CHANNELS - Ch1: ");
    Serial.print(rc_channels.chan1_raw);
    Serial.print(", Ch2: ");
    Serial.print(rc_channels.chan2_raw);
    Serial.print(", Ch3: ");
    Serial.print(rc_channels.chan3_raw);
    Serial.print(", Ch4: ");
    Serial.println(rc_channels.chan4_raw);
    break;
  }

  default:
    // Unknown message; already printed basic info
    break;
  }
}

void mavlinkReceiveLoop()
{
  // Read available data from the input serial port
  if (inputSerial.available() > 0)
  {
    uint8_t byte = inputSerial.read();
    mavlink_message_t msg;
    mavlink_status_t status;

    // Parse the incoming byte
    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
    {
      // A complete message has been received
      processMessage(&msg);
    }
  }
}

void mavlinkSenderLoop()
{
  static unsigned long lastSendTime = 0;

  if (millis() - lastSendTime >= MAVLINK_SEND_INTERVAL_MSEC)
  {
    lastSendTime = millis();

    // Send HEARTBEAT
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    heartbeat.type = MAV_TYPE_GCS;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    mavlink_msg_heartbeat_encode(100, 1, &msg, &heartbeat);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    inputSerial.write(buf, len);
    Serial.println("Sent HEARTBEAT");

    // Send RC_CHANNELS_OVERRIDE
    mavlink_rc_channels_override_t rc;
    rc.target_system = 1;
    rc.target_component = 1;

    double c1 = 1500 + 500 * sin(millis() / 1000.0);
    double c2 = 1500 + 500 * cos(millis() / 1000.0);
    rc.chan1_raw = c1;
    rc.chan2_raw = c2;
    rc.chan3_raw = 2000;
    rc.chan4_raw = 1800;

    mavlink_msg_rc_channels_override_encode(100, 1, &msg, &rc);
    uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
    uint16_t len2 = mavlink_msg_to_send_buffer(buf2, &msg);
    inputSerial.write(buf2, len2);
    Serial.println("Sent RC_CHANNELS_OVERRIDE");
  }
}
