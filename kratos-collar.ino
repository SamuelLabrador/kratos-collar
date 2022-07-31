#include <Wire.h>
#include "icp101xx.h"
#include <queue>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// 3 on RP2040 connect

#define CLAMP_PWR 12
#define CLAMP_PIN 13

BluetoothSerial SerialBT;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
ICP101xx icp;

typedef struct EventPacket {
  // [ code, eventID ]
  unsigned char data[2];  // SHOULD ALWAYS BE 1
} EventPacket;

typedef struct XLPacket {
  unsigned char code; // SHOULD ALWAYS BE 1
  unsigned long timestamp;
  float data[7]; // XL + QUATERNION
} XLPacket;

typedef struct PressureTempPacket {
  unsigned char code; // SHOULD ALWAYS BE 2
  float data[2]; // PRESSURE + TEMPERATURE
} PressureTempPacket;

typedef struct BluetoothPacket {
  unsigned char code;
  float data[3]; // UP TO 12 values depending on the code
} BluetoothPacket;

const int MA_AMOUNT = 5;

#define EVENT_PACKET 0
#define XL_PACKET 1
#define PT_PACKET 2

enum XL_TRANSMIT_STATE {
  WAIT_TRANSMIT,
  TRANSMIT
};

/**
 * GLOBAL VARIABLES
 */
bool closedState = false;
XL_TRANSMIT_STATE transmitState = WAIT_TRANSMIT;
const uint8_t STOP_DURATION = 600;
const uint8_t START_DURATION = 15;

// BLE Buffers
const uint8_t xlBufferSize = 100;
const uint8_t pressureSize = 40;
XLPacket xlBuffer[xlBufferSize] = {};
//QuaternionPacket quaternionBuffer[xlBufferSize] = {};
BluetoothPacket pressureTempBuffer[pressureSize] = {};
unsigned long pCount = 0;

QueueHandle_t bluetoothQueue;

/**
 * HELPER FUNCTIONS
 */
void shiftLeft(XLPacket *buffer, uint8_t window_size) {
  for (uint8_t i = 0; i < window_size - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
}
//void shiftLeft(QuaternionPacket *buffer, uint8_t window_size) {
//  for (uint8_t i = 0; i < window_size - 1; i++) {
//    buffer[i] = buffer[i + 1];
//  }
//}
void shiftLeft(PressureTempPacket *buffer, uint8_t window_size) {
  for (uint8_t i = 0; i < window_size - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
}
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

/**
 * CONFIGURATION FUNCTIONS
 */
void configureBarometer() {
   Serial.println("Configuring barometer");
  if (! icp.begin()) {
    while (1) yield();
  }
   Serial.println("Barometer initialized!");
}

void configureAccel() {
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("IMU initialized");
}

void configureBluetooth() {
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

/**
 * TASK FUNCTIONS
 */
void bluetoothTransmitTask(void * params) {
  struct BluetoothPacket bluetoothPacket;
  for (;;) {
    if (bluetoothQueue != NULL) {
      if (xQueueReceive(bluetoothQueue, &(bluetoothPacket), ( TickType_t ) 1 ) == pdPASS) {
          const byte * p = (const byte*) &bluetoothPacket;
          for (uint8_t i = 0; i < sizeof bluetoothPacket; i++) {
            SerialBT.write(*p++);
          }
      }
    }
  }
}

void pressureTempTask(void * params) {
  /* The parameter value is expected to be 1 as 1 is passed in the
  pvParameters value in the call to xTaskCreate() below. */

  uint8_t currentPTBufferSize = pressureSize;
  for ( ;; ) {
    if (SerialBT.hasClient()) {
      while (SerialBT.hasClient()) {
        icp.measure(ICP101xx::ACCURATE); // ~12 ms delay here.

        float pressure = icp.getPressurePa();
        float temperature = icp.getTemperatureC();

        if (currentPTBufferSize <= 0) {
          shiftLeft(pressureTempBuffer, pressureSize);
        } else {
          currentPTBufferSize--;
        }

        // Build Pressure temp packet
        BluetoothPacket packet = {
          PT_PACKET,
          pressure,
          temperature,
          0.0
        };

        // Update the buffer
        pressureTempBuffer[pressureSize - 1] = packet;

        if (transmitState == TRANSMIT) {
          xQueueSend(bluetoothQueue,
                     (void*)&packet,
                     ( TickType_t ) 0);
          pCount++;
        }
      }
    }
  }
}

void mainTask(void* params) {
  /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. */

  for( ;; ) {
    if (SerialBT.hasClient()) {
      Serial.println("Client is connected!");;
      Serial.print("Amount of ticks between rest: ");
      float result;

      sensors_event_t linearXL, gyro, magnetometer;
      double previousAcceleration = -1.0;

      uint8_t loadCount = 0;
      float movingAverageXL[MA_AMOUNT] = {};
      float previousAverage = -1.0;
      bool icpMeasuring = false;
      uint8_t stopCount = STOP_DURATION;
      uint8_t startCount = START_DURATION;

      uint8_t currentXLBufferSize = xlBufferSize;

      int xlCountDebug = 0;

      transmitState = WAIT_TRANSMIT;

      unsigned short xlCount = 0;
      unsigned short barometerCount = 0;
      unsigned long start;
      unsigned long cycleStart;
      unsigned long totalCount = 0;
      TickType_t xLastWakeTime;
      xLastWakeTime = xTaskGetTickCount();
      const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;

      while (SerialBT.hasClient()) {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        cycleStart = millis();

        totalCount += 1;

        // Read xl and gyr data every 10ms. 100Hz.
        if (true) {

          bno.getEvent(&linearXL, Adafruit_BNO055::VECTOR_LINEARACCEL);
          bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
          bno.getEvent(&magnetometer, Adafruit_BNO055::VECTOR_MAGNETOMETER);

          // Units m/s^2
          float xl_x = linearXL.acceleration.x;
          float xl_y = linearXL.acceleration.y;
          float xl_z = linearXL.acceleration.z;

          float magnitude = sqrt(xl_x * xl_x + xl_y * xl_y + xl_z * xl_z);

          float average = shiftAndPop(movingAverageXL, MA_AMOUNT, magnitude);

          // Serial.print("Transmit State: ");
          // Serial.print(transmitState);
          // Serial.print(" The difference: ");
          // Serial.print(abs(average - previousAverage));
          // Serial.print(" The stop count: ");
          // Serial.println(stopCount);

          if (loadCount < MA_AMOUNT) {
            movingAverageXL[loadCount] = average;
            loadCount += 1;
          } else {
            if (closedState) {
              switch (transmitState) {
                case WAIT_TRANSMIT:
                  if (previousAverage >= 0 && abs(average - previousAverage) > 0.05) {
                    startCount -= 1;
                  } else {
                    startCount = START_DURATION;
                  }

                  if (startCount <= 0) {
                    transmitState = TRANSMIT;
                    Serial.println("Starting transmission!");
                    start = millis();
                    totalCount = 0;
                    // TODO: Adjust for Bluetooth classic
                    //                  EventCharacteristic.writeValue(0x01);

                    // TODO: Adjust for Bluetooth classic
//                  for (uint8_t i = xlBufferSize; i < xlBufferSize; i++) {
                    //                    LinearAccelerationCharacteristic.writeValue(xlBuffer[i]);
                    //                    QuaternionCharacteristic.writeValue(quaternionBuffer[i]);
                    //                    LinearAccelerationCharacteristic.valueUpdated();
                    //
                    //                    if (i < pressureSize) {
                    //                      PressureTempCharacteristic.writeValue(pressureTempBuffer[i]);
                    //                    }
//                  }
                  }
                  break;
                case TRANSMIT:
                  if (abs(average - previousAverage) < 0.1) {
                    stopCount -= 1;
                  } else {
                    stopCount = STOP_DURATION;
                  }

                  if (stopCount <= 0 || !closedState) {
                    Serial.println("Stopping transmission!");
                    transmitState = WAIT_TRANSMIT;
                    Serial.print("Elapsed time: ");
                    Serial.println(millis() - start);
                    Serial.print("Total cycles: ");
                    Serial.println(totalCount);
                    stopCount = 0;

                    // TODO: Adjust for Bluetooth classic
                    //                  EventCharacteristic.writeValue(0x00);

                    Serial.print("XL and Quat Count: ");
                    Serial.print(xlCountDebug);
                    Serial.print(" Pressure Count: ");
                    Serial.println(pCount);

                    xlCountDebug = 0;
                    pCount = 0;
                  }
                  break;
              }
            }
            previousAverage = average;
          }

          // Build XL Packet
          uint64_t zero = 0;
          XLPacket packet = {xl_x, xl_y, xl_z};

          // Build Quaternion packet
//          float quat_w = quaternion.w();
//          float quat_x = quaternion.x();
//          float quat_y = quaternion.y();
//          float quat_z = quaternion.z();
//          QuaternionPacket quatPacket = {quat_w, quat_x, quat_y, quat_z};

          if (currentXLBufferSize <= 0) {
            // Don't pop values until we've filled up the buffer
//            shiftLeft(quaternionBuffer, xlBufferSize);
          } else {
            currentXLBufferSize--;
          }

//          quaternionBuffer[xlBufferSize - 1] = quatPacket;
          xlBuffer[xlBufferSize - 1] = packet;

          if (transmitState == TRANSMIT) {
            // TODO: Adjust for Bluetooth classic
            //            LinearAccelerationCharacteristic.writeValue(packet);
            //            QuaternionCharacteristic.writeValue(quatPacket);
            xlCountDebug++;
          }
        } else {
          xlCount++;
        }
      }
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void clampCheckLoop(void* params) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;

  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    closedState = digitalRead(CLAMP_PIN);
  }
}

TaskHandle_t xHandle = NULL;
TaskHandle_t ptHandle = NULL;
TaskHandle_t clampHandle = NULL;
TaskHandle_t bluetoothHandle = NULL;

/**
 * STARTUP LOGIC
 */
void setup() {
  Serial.begin(115200);

  vTaskStartScheduler();
  bluetoothQueue = xQueueCreate(600, sizeof(BluetoothPacket));


  Serial.println("Collar warming up...");

  configureBluetooth();
  configureAccel();
  configureBarometer();

  pinMode(CLAMP_PIN, INPUT);
  pinMode(CLAMP_PWR, OUTPUT);
  digitalWrite(CLAMP_PWR, HIGH);

  // Accelerometer task
  xTaskCreate(
    mainTask,        /* Function that implements the task. */
    "MAIN_TASK",          /* Text name for the task. */
    2000,      /* Stack size in words, not bytes. */
    ( void * ) 1,    /* Parameter passed into the task. */
    4,/* Priority at which the task is created. */
    &xHandle );      /* Used to pass out the created task's handle. */

  // Pressure-Temperature Task
  xTaskCreate(
    pressureTempTask,
    "PRESSURE_TEMP_TASK",
    2000,
    ( void* ) 1,
    4,
    &ptHandle
  );

  // Clamp detection Task
  xTaskCreate(
    clampCheckLoop,
    "CLAMP_TASK",
    2000,
    ( void* ) 1,
    4,
    &clampHandle
  );

  xTaskCreate(
    bluetoothTransmitTask,
    "BLUETOOTH_TASK",
    2000,
    ( void* ) 1,
    tskIDLE_PRIORITY,
    &bluetoothHandle
  );

  Serial.println("Finished collar startup!");
}


// Main loop is empty. Logic should be running in tasks.
void loop() {}
