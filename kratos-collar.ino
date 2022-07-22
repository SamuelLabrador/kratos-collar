#include <ArduinoBLE.h>
#include <Wire.h>
#include "icp101xx.h"
#include <Scheduler.h>
#include <queue>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
ICP101xx icp;

typedef struct XLPacket {
  float data[3];
} XLPacket;

typedef struct GravityPacket {
  float data[3];
} GravityPacket;


typedef struct Quaternion {
  float data[4];
} Quaternion;

typedef struct PressureTempPacket {
  float data[2];
} PressureTempPacket;

BLEService BarbellService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service
BLETypedCharacteristic<byte> EventCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1217", BLERead | BLENotify);
BLETypedCharacteristic<int> VoltageCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1218", BLERead | BLENotify);
BLETypedCharacteristic<XLPacket> LinearAccelerationCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify);
BLETypedCharacteristic<Quaternion> QuaternionCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1219", BLERead | BLENotify);
BLETypedCharacteristic<PressureTempPacket> PressureTempCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1220", BLERead | BLENotify);
BLETypedCharacteristic<GravityPacket> GravityVectorCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1221", BLERead | BLENotify);

const int MA_AMOUNT = 5;

enum XL_TRANSMIT_STATE {
  WAIT_TRANSMIT,
  TRANSMIT
};

bool closedState = false;
XL_TRANSMIT_STATE transmitState = WAIT_TRANSMIT;
uint8_t command;

double shiftAndPop(float *window, uint8_t window_size, double element) {
  float average = 0;
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

  BarbellService.addCharacteristic(LinearAccelerationCharacteristic);
  BarbellService.addCharacteristic(QuaternionCharacteristic);
  BarbellService.addCharacteristic(PressureTempCharacteristic);
  BarbellService.addCharacteristic(EventCharacteristic);
  BarbellService.addCharacteristic(VoltageCharacteristic);

  // Set default values for characteristics
  LinearAccelerationCharacteristic.writeValue({0.0, 0.0, 0.0});
  QuaternionCharacteristic.writeValue({0.0, 0.0, 0.0, 0.0});
  PressureTempCharacteristic.writeValue({0.0, 0.0});
  GravityVectorCharacteristic.writeValue({0.0, 0.0, 0.0});
  VoltageCharacteristic.writeValue(0.0);
  EventCharacteristic.writeValue(0x00);

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

  Serial.println("Battery meter initialized!");
}

void configureAccel() {
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
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

    sensors_event_t linearXL, gravity;
    double previousAcceleration = -1.0;

    uint8_t previousCommand = 0x00;

    uint8_t loadCount = 0;
    float movingAverageXL[MA_AMOUNT] = {};
    float previousAverage = -1.0;
    const uint8_t STOP_DURATION = 600;
    const uint8_t START_DURATION = 15;
    bool icpMeasuring = false;
    uint8_t stopCount = STOP_DURATION;
    uint8_t startCount = START_DURATION;

    transmitState = WAIT_TRANSMIT;

    while (central.connected()) {
      closedState = digitalRead(3);

      bno.getEvent(&linearXL, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&gravity, Adafruit_BNO055::VECTOR_GRAVITY);
      imu::Quaternion quaternion = bno.getQuat();

      float quat_w = quaternion.w();
      float quat_x = quaternion.x();
      float quat_y = quaternion.y();
      float quat_z = quaternion.z();

      // Units m/s^2
      float xl_x = linearXL.acceleration.x;
      float xl_y = linearXL.acceleration.y;
      float xl_z = linearXL.acceleration.z;

      float magnitude = sqrt(xl_x * xl_x + xl_y * xl_y + xl_z * xl_z);

      float average = shiftAndPop(movingAverageXL, MA_AMOUNT, magnitude);

      Serial.print("Transmit State: ");
      Serial.print(transmitState);
      Serial.print(" The difference: ");
      Serial.print(abs(average - previousAverage));
      Serial.print(" The stop count: ");
      Serial.println(stopCount);

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
              EventCharacteristic.writeValue(0x01);
            }
            break;
          case TRANSMIT:
            if (abs(average - previousAverage) < 0.1) {
              stopCount -= 1;
            } else {
              stopCount = STOP_DURATION;
            }

            if (stopCount <= 0 || !closedState) {
              transmitState = WAIT_TRANSMIT;
              stopCount = 0;
              EventCharacteristic.writeValue(0x00);
            }
            break;
        }
        previousAverage = average;
      }

      if (!icpMeasuring) {
        icp.measureStart(ICP101xx::ACCURATE);
        icpMeasuring = true;
      }

      if (icp.dataReady()) {
        float pressure = icp.getPressurePa();
        float temperature = icp.getTemperatureC();

        if (transmitState == TRANSMIT) {
          PressureTempPacket ptPacket = {pressure, temperature};
          PressureTempCharacteristic.writeValue(ptPacket);
        }
        icpMeasuring = false;
      }


      if (transmitState == TRANSMIT) {
        uint64_t zero = 0;

        XLPacket packet = {xl_x, xl_y, xl_z};
        LinearAccelerationCharacteristic.writeValue(packet);

        Quaternion quatPacket = {quat_w, quat_x, quat_y, quat_z};
        QuaternionCharacteristic.writeValue(quatPacket);
      }

      previousCommand = command;
      // Wait 10 milliseconds before getting the reading again
      delay(20);
    }

    digitalWrite(LED_BUILTIN, LOW);

    delay(250);
  }
}

void clampCheckLoop() {
  closedState = digitalRead(3);
  delay(500);
}

void ledLoop() {
  const unsigned long amt = 250;
  if (BLE.connected()) {
    if (closedState) {
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
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(amt * 2);
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(amt);
}

void batteryLoop() {
  if (BLE.connected()) {
    int value = analogRead(A2);
    VoltageCharacteristic.writeValue(value);
  }

  delay(5000);  // dont query too often!
}

void setup() {
  Serial.begin(9600);
//  while (!Serial) {}
  Serial.println("Collar warming up...");
  pinMode(LED_BUILTIN, OUTPUT);

  configureBLE();
  configureAccel();
  configureBarometer();
  pinMode(3, INPUT);

  Scheduler.startLoop(clampCheckLoop);
  Scheduler.startLoop(ledLoop);
  Scheduler.startLoop(batteryLoop);
  Scheduler.startLoop(mainLoop);
  Serial.println("Finished collar startup!");
  delay(1000);
}

void loop() {
  yield();
}
