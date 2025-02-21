#include <main.h>

// Define the input serial port (e.g., UART connected to MAVLink source)
HardwareSerial &inputSerial = Serial1;

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup()
{
  // Initialize pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  analogWrite(GREEN_PIN, 255); // Green
  analogWrite(BLUE_PIN, 255);  // Blue
  analogWrite(RED_PIN, 0);     // Red

  // Initialize debugging serial port
  Serial.begin(115200);
  // while (!Serial)
  // {
  //   ; // Wait for serial connection (optional)
  // }

  // Initialize MAVLink input serial port
  inputSerial.begin(115200); // Match the baud rate of your MAVLink source
  Serial.println("MAVLink Reader Started");
  analogWrite(RED_PIN, 255); // Red

  // begin initialization
  if (!BLE.begin())
  {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1)
      ;
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
}

void processMessage(mavlink_message_t *msg)
{
  // Handle the message based on its ID
  Serial.print("Received message ID: ");
  Serial.print(msg->msgid);
  Serial.print(" from System ID: ");
  Serial.print(msg->sysid);
  Serial.print(", Component ID: ");
  Serial.println(msg->compid);

  // Example: Handle specific message types
  switch (msg->msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
  {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(msg, &heartbeat);
    Serial.print("  HEARTBEAT - Type: ");
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
  default:
    // Unknown message; already printed basic info
    break;
  }
}

void loop()
{
  heartBeatLED();

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

      // analogWrite(GREEN_PIN, 255); // Green
    }
  }

  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central)
  {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected())
    {
      if (switchCharacteristic.written())
      {
        if (switchCharacteristic.value())
        {
          Serial.println("LED on");
          analogWrite(BLUE_PIN, 0);
        }
        else
        {
          Serial.println(F("LED off"));
          analogWrite(BLUE_PIN, 255);
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
