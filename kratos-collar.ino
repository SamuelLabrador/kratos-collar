#include "LSM6DSOX_CUSTOM.h"
#include <ArduinoBLE.h>
//#include <Adafruit_DPS310.h>
#include <Wire.h>
#include "icp101xx.h"
#include <Scheduler.h>
#include <Adafruit_LC709203F.h>
#include <queue>

Adafruit_LC709203F lc;
typedef struct Packet {
  uint16_t value[6];
  float barometerData[2];
}Packet;
BLEService BarbellService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service
BLETypedCharacteristic<Packet> DataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify | BLEWrite);
//BLETypedCharacteristic<byte> CommandCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1216", BLEWrite);
BLETypedCharacteristic<byte> EventCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1217", BLERead | BLENotify);
BLETypedCharacteristic<int> VoltageCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1218", BLERead | BLENotify);


double GRAVITY = 9.80665;
const int MA_AMOUNT = 5;

enum COMMAND {
  SLEEP = 0x00,
  READY = 0x01
};

enum XL_TRANSMIT_STATE {
  WAIT_TRANSMIT,
  TRANSMIT
};

XL_TRANSMIT_STATE transmitState = WAIT_TRANSMIT;
ICP101xx icp;
uint8_t command;

double shiftAndPop(double *window, uint8_t window_size, double element) {
  double average = 0;
  for (uint8_t i = 0; i < window_size - 1; i++) {
    window[i] = window[i + 1];
    average += window[i];
  }

  window[window_size - 1] = element;
  average += element;
  return average / (double)window_size;
}

void configureBLE() {
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Kratos Collar");
  BLE.setAdvertisedService(BarbellService);
  BarbellService.addCharacteristic(DataCharacteristic);
//  BarbellService.addCharacteristic(CommandCharacteristic);
  BarbellService.addCharacteristic(EventCharacteristic);
  EventCharacteristic.setValue(0x00);
  BarbellService.addCharacteristic(VoltageCharacteristic);

  // add service
  BLE.addService(BarbellService);

  // start advertising
  BLE.advertise();
  Serial.println("BLE Peripheral initialized.");
}

void configureBarometer() {
  Serial.println("Configuring barometer");
  if (! icp.begin()) {
    while (1) yield();
  }
  Serial.println("Barometer initialized!");
}

void configureBatteryMeter() {
  Serial.println("Configuring battery meter");

  if (!lc.begin()) {
    Serial.println("Couldnt find Adafruit LC709203F?\nMake sure a battery is plugged in!");
    while (1) yield();
  }

  lc.setThermistorB(3950);
  lc.setPackSize(LC709203F_APA_1000MAH);
  lc.setAlarmVoltage(3.8);

  Serial.println("Battery meter initialized!");
}

void configureAccel() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("IMU initialized");
}

void mainLoop() {
  BLEDevice central = BLE.central();

  float result;
  icp.measure(ICP101xx::ACCURATE);

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    uint16_t raw_xl_x, raw_xl_y, raw_xl_z;
    uint16_t gyro_x, gyro_y, gyro_z;
    double previousAcceleration = -1.0;

    uint8_t previousCommand = 0x00;
    bool icpMeasuring = false;

    uint8_t loadCount = 0;
    double movingAverageXL[MA_AMOUNT] = {};
    double previousAverage = -1.0;
    const uint8_t STOP_DURATION = 100;
    uint8_t stopCount = STOP_DURATION;
    const uint8_t START_DURATION = 10;
    uint8_t startCount = START_DURATION;
    transmitState = WAIT_TRANSMIT; // Should be WAIT_TRANSMIT

    while (central.connected()) {
      if (!icpMeasuring) {
        icp.measureStart(ICP101xx::ACCURATE);
        icpMeasuring = true;
      }

      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && icp.dataReady()) {
        float pressure = icp.getPressurePa() / 100.0;
        float temperature = icp.getTemperatureC();
        IMU.readAcceleration(raw_xl_x, raw_xl_y, raw_xl_z);
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

        double xl_x = (raw_xl_x * (4.0 / 32768.0)) * GRAVITY;
        double xl_y = (raw_xl_y * (4.0 / 32768.0)) * GRAVITY;
        double xl_z = (raw_xl_z * (4.0 / 32768.0)) * GRAVITY;

        double magnitude = sqrt(xl_x * xl_x + xl_y * xl_y + xl_z * xl_z);

        double average = shiftAndPop(movingAverageXL, MA_AMOUNT, magnitude);

        if (loadCount < MA_AMOUNT) {
          movingAverageXL[loadCount] = average;
          loadCount += 1;
        } else {
          switch (transmitState) {
            case WAIT_TRANSMIT:
              if (previousAverage >= 0 && abs(average - previousAverage) > 0.1) {
                startCount -= 1;
              } else {
                startCount = START_DURATION;
              }

              if (startCount <= 0) {
                transmitState = TRANSMIT;
                EventCharacteristic.setValue(0x01);
              }
              break;
            case TRANSMIT:
              if (abs(average - previousAverage) < 0.01) {
                stopCount -= 1;
              } else {
                stopCount = STOP_DURATION;
              }

              if (stopCount <= 0) {
                transmitState = WAIT_TRANSMIT;
                EventCharacteristic.setValue(0x00);
              }
              break;
          }
        }

        previousAverage = average;

        if (transmitState == TRANSMIT) {
          uint64_t zero = 0;

          Packet packet = {raw_xl_x, raw_xl_y, raw_xl_z,
                           gyro_x, gyro_y, gyro_z,
                           pressure, temperature};

          DataCharacteristic.writeValue(packet);
        }

        icpMeasuring = false;
      }

      previousCommand = command;
      yield();
    }

    digitalWrite(LED_BUILTIN, LOW);

    delay(250);
  }
}

void ledLoop() {
  const unsigned long amt = 250;
  if (BLE.connected()) {
    switch (transmitState) {
      case WAIT_TRANSMIT:
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case TRANSMIT:
        digitalWrite(LED_BUILTIN, LOW);
        delay(amt);
        digitalWrite(LED_BUILTIN, HIGH);
        break;
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(amt);
}

void batteryLoop() {
  if (BLE.connected()) {
    int value = analogRead(A2);
    VoltageCharacteristic.setValue(value);
  }

  delay(5000);  // dont query too often!
}

void activityDetectionLoop() {

}

void setup() {
  Serial.begin(9600);
//  while (!Serial) {}
  pinMode(LED_BUILTIN, OUTPUT);

  configureBLE();
  configureAccel();
  configureBarometer();

  Scheduler.startLoop(ledLoop);
  Scheduler.startLoop(mainLoop);
  Scheduler.startLoop(batteryLoop);
}

void loop() {
  yield();
}
