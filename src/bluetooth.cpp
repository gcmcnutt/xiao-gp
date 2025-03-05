#include <main.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void blueToothSetup()
{
  // begin initialization
  if (!BLE.begin())
  {
    logPrint(ERROR, "starting Bluetooth® Low Energy module failed!");

    // TODO don't hang here
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

  logPrint(INFO, "BLE LED Peripheral");
}

// polling loop for BT work
void blueToothLoop()
{
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral
  if (central)
  {
    logPrint(DEBUG, "Connected to central: %s", central.address());

    // while the central is still connected to peripheral:
    while (central.connected())
    {
      if (switchCharacteristic.written())
      {
        if (switchCharacteristic.value())
        {
          analogWrite(BLUE_PIN, 0);
          logPrint(INFO, "LED on");
        }
        else
        {
          analogWrite(BLUE_PIN, 255);
          logPrint(INFO, "LED off");
        }
      }
    }

    // when the central disconnects, print it out:
    logPrint(DEBUG, "Disconnected from central: %s", central.address());
  }
}