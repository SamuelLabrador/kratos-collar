#include "LSM6DSOX_CUSTOM.h"
#include <Adafruit_BMP3XX.h>
#include <ArduinoBLE.h>

typedef struct Packet {
  uint16_t value[6];
  float barometerData[2];
}Packet;
BLEService BarbellService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service
BLETypedCharacteristic<Packet> DataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify | BLEWrite);
BLETypedCharacteristic<byte> CommandCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1216", BLEWrite);
//BLETypedCharacteristic<byte> EventCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1217", BLERead | BLENotify);

enum COMMAND {
  WAIT = 0x00,
  TRANSMIT_DATA = 0x01
};

Adafruit_BMP3XX bmp;
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
//  BarbellService.addCharacteristic(EventCharacteristic);

  // add service
  BLE.addService(BarbellService);

  // start advertising
  BLE.advertise();
  Serial.println("BLE LED Peripheral, waiting for connections....");
}

void configureBarometer() {
  if (! bmp.begin_SPI(10)) {
    Serial.println("Failed to find DPS");
    while (1) yield();
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
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
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    uint16_t xl_x, xl_y, xl_z;
    uint16_t gyro_x, gyro_y, gyro_z;
    uint8_t previousCommand = 0x00;
    uint8_t command;
    if (central.connected()) {
      digitalWrite(LED_BUILTIN, HIGH);
    }

    while (central.connected()) {
      CommandCharacteristic.readValue(command);

      switch (command) {
        case WAIT:
          // We are moving from TRANSMIT_DATA -> WAIT
          // Set EventCharacteristic -> 0x01
          if (previousCommand == TRANSMIT_DATA) {
//            EventCharacteristic.setValue(0x01);
          }
          break;
        case TRANSMIT_DATA:
          if (previousCommand == WAIT) {
//            EventCharacteristic.setValue(0x00);
          }

          if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){ // && bmp.performReading()) {
            float pressure = 0.0; //bmp.readPressure() / 100.0;
            float temperature = 0.0; //bmp.readTemperature();
            IMU.readAcceleration(xl_x, xl_y, xl_z);
            IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
            uint64_t zero = 0;

            Packet packet = {xl_x, xl_y, xl_z,
                             gyro_x, gyro_y, gyro_z,
                             pressure, temperature};

            DataCharacteristic.writeValue(packet);
          }
          break;
      }

      previousCommand = command;
    }


    digitalWrite(LED_BUILTIN, LOW);

    delay(500);
  }
}
        // Uncomment for debug
//        Serial.print(F("Pressure = "));
//        Serial.print(pressure);
//        Serial.print(" hPa ");
//
//        Serial.print(F("Temperature = "));
//        Serial.print(temperature);
//        Serial.print(" *C ");
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
//        Serial.println("");
