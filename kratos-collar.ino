#include "LSM6DSOX_CUSTOM.h"
#include <Adafruit_DPS310.h>
#include <ArduinoBLE.h>
#include <Scheduler.h>

#define DPS310_CS 10

typedef struct packet {
  uint16_t value[6];
  float barometerData[2];
}packet;
BLEService BarbellService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service
BLETypedCharacteristic<packet> DataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify | BLEWrite);
BLETypedCharacteristic<byte> CommandCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1216", BLEWrite);

enum COMMAND {
  WAIT = 0x00,
  TRANSMIT_DATA = 0x01
};

Adafruit_DPS310 dps;
bool transmitData;

void configureBLE() {
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Kratos Collar");
  BLE.setAdvertisedService(BarbellService);
  BarbellService.addCharacteristic(DataCharacteristic);
  BarbellService.addCharacteristic(CommandCharacteristic);

  // add service
  BLE.addService(BarbellService);

  // start advertising
  BLE.advertise();
  Serial.println("BLE LED Peripheral, waiting for connections....");
}

void configureBarometer() {
  if (! dps.begin_SPI(10)) {
    Serial.println("Failed to find DPS");
    while (1) yield();
  }

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}

void configureAccel() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

// Clk @ 133Mhz
void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(10, OUTPUT);

  configureBLE();
  configureAccel();
  configureBarometer();
}

void loop() {
  sensors_event_t temp_event, pressure_event;

  BLEDevice central = BLE.central();
  boolean flag = false;

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      flag = true;
      digitalWrite(LED_BUILTIN, HIGH);

      uint16_t xl_x, xl_y, xl_z;
      uint16_t gyro_x, gyro_y, gyro_z;

      uint8_t command;
      CommandCharacteristic.readValue(command);

      dps.getEvents(&temp_event, &pressure_event);

      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(xl_x, xl_y, xl_z);
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
        uint64_t zero = 0;

        packet packet = {xl_x, xl_y, xl_z,
                         gyro_x, gyro_y, gyro_z,
                         pressure_event.pressure, temp_event.temperature};

        // Uncomment for debug
        Serial.print(F("Pressure = "));
        Serial.print(pressure_event.pressure);
        Serial.print(" hPa ");

        Serial.print(F("Temperature = "));
        Serial.print(temp_event.temperature);
        Serial.print(" *C ");

//
//        Serial.print("XL = ");
//        Serial.print(xl_x);
//        Serial.print(", ");
//        Serial.print(xl_y);
//        Serial.print(", ");
//        Serial.print(xl_z);
//        Serial.print(", ");
//
//        Serial.print("GYR = ");
//        Serial.print(gyro_x);
//        Serial.print(", ");
//        Serial.print(gyro_y);
//        Serial.print(", ");
//        Serial.print(gyro_z);
//        Serial.println(", ");
        Serial.println("");

        DataCharacteristic.writeValue(packet);
      }
//      switch (command) {
//        case WAIT:
//          delay(10);
//          break;
//        case TRANSMIT_DATA:
//          if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
//            IMU.readAcceleration(xl_x, xl_y, xl_z);
//            IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
//            uint64_t zero = 0;
//
//            packet packet = {xl_x, xl_y, xl_z,
//                             gyro_x, gyro_y, gyro_z,
//                             temp_event.temperature, pressure_event.pressure};
//
//            DataCharacteristic.writeValue(packet);
//          }
//          break;
//      }

      yield();
    }

    if (flag) {
      Serial.println("Disconnected!");
      digitalWrite(LED_BUILTIN, LOW);
    }

    delay(500);
  }
}
