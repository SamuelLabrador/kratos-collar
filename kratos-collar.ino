#include <Wire.h>
#include "icp101xx.h"
#include <queue>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "BluetoothSerial.h"
#include <ArduinoJson.h>

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

const int MA_AMOUNT = 5;

#define EVENT_PACKET 'E'
#define XL_PACKET "X'
#define PT_PACKET "P"
#define EVENT_TRANSMIT_START 1.0
#define EVENT_TRANSMIT_END 2.0

enum XL_TRANSMIT_STATE {
  WAIT_TRANSMIT,
  TRANSMIT
};

struct PressurePacket {
  float data[2];
};

struct IMUPacket {
  float data[12];
};

struct EventPacket {
  unsigned char event;
};

/**
 * GLOBAL VARIABLES
 */
bool closedState = false;
XL_TRANSMIT_STATE transmitState = WAIT_TRANSMIT;
const uint16_t STOP_DURATION = 350;
const uint16_t START_DURATION = 15;

// Bluetooth Buffers
const uint16_t IMU_BUFFER_SIZE = 100;
const uint16_t PRESSURE_BUFFER_SIZE = 40;

IMUPacket imuBuffer[IMU_BUFFER_SIZE];
PressurePacket pressureTempBuffer[PRESSURE_BUFFER_SIZE];
unsigned long pCount = 0;

// FreeRTOS variables
QueueHandle_t pressureQueue;
QueueHandle_t imuQueue;
QueueHandle_t eventQueue;

TaskHandle_t xHandle = NULL;
TaskHandle_t ptHandle = NULL;
TaskHandle_t clampHandle = NULL;
TaskHandle_t bluetoothHandle = NULL;

/**
 * HELPER FUNCTIONS
 */
double shiftAndPop(float *window, uint16_t window_size, double element) {
  float average = 0;
  for (uint8_t i = 0; i < window_size - 1; i++) {
    window[i] = window[i + 1];
    average += window[i];
  }

  window[window_size - 1] = element;
  average += element;
  return average / (double)window_size;
}

void shiftLeft(IMUPacket *buffer, uint16_t size) {
  for (uint8_t i = 0; i < size - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
}

void shiftLeft(PressurePacket *buffer, uint16_t size) {
  for (uint8_t i = 0; i < size - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
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
  SerialBT.begin("KratosCollar"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

/**
 * TASK FUNCTIONS
 */
void bluetoothTransmitTask(void * params) {
  delay(100);
  for (;;) {
    PressurePacket pressurePacket;
    IMUPacket imuPacket;
    EventPacket eventPacket;
    if (pressureQueue != NULL && imuQueue != NULL && eventQueue != NULL) {
      String payload ="";

      if (xQueueReceive(eventQueue, &(eventPacket), (TickType_t) 0) == pdPASS) {
        StaticJsonDocument<256> eventDoc;
        eventDoc["e"] = "E";
        eventDoc["d"] = eventPacket.event;

        serializeJson(eventDoc, payload);
      }

      for(uint8_t j = 0; j < 10; j++) {
        if (xQueueReceive(pressureQueue, &(pressurePacket), ( TickType_t ) 0) == pdPASS) {
          DynamicJsonDocument pressureDoc(2048);
          pressureDoc["e"] = "P";
          for (uint8_t i = 0; i < 3; i++) {
            pressureDoc["d"][i] = pressurePacket.data[i];
          }

          serializeJson(pressureDoc, payload);
        }

      }

      for(uint8_t j = 0; j < 10; j++) {
        if (xQueueReceive(imuQueue, &(imuPacket), (TickType_t) 0) == pdPASS) {
          DynamicJsonDocument imuDoc(2048);
          imuDoc["e"] = "I";
          for (uint8_t i = 0; i < 9; i++) {
            imuDoc["d"][i] = imuPacket.data[i];
          }
//          serializeJson(imuDoc, Serial);
//          Serial.println("");
          serializeJson(imuDoc, payload);
        }
      }

      if (payload.length() > 0) {
        uint8_t buf[payload.length()];
        memcpy(buf, payload.c_str(), payload.length());
        SerialBT.write(buf, payload.length());
      }
    }
  }
}

void pressureTempTask(void * params) {
  delay(50);
  /* The parameter value is expected to be 1 as 1 is passed in the
  pvParameters value in the call to xTaskCreate() below. */

  uint8_t currentPTBufferSize = PRESSURE_BUFFER_SIZE;
  for ( ;; ) {
    if (SerialBT.hasClient()) {
      while (SerialBT.hasClient()) {
        icp.measure(ICP101xx::ACCURATE); // ~25 ms delay here.

        float pressure = icp.getPressurePa();
        float temperature = icp.getTemperatureC();

        PressurePacket pp = PressurePacket {
          pressure, temperature
        };

        // Update the buffer
        shiftLeft(pressureTempBuffer, PRESSURE_BUFFER_SIZE);
        pressureTempBuffer[PRESSURE_BUFFER_SIZE - 1] = pp;

        if (transmitState == TRANSMIT) {
          xQueueSend(pressureQueue,
                     (void*)&pp,
                     ( TickType_t ) 0);
          pCount++;
        }
      }
    }
  }
}

void mainTask(void* params) {
  delay(50);
  /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. */

  for( ;; ) {
    if (SerialBT.hasClient()) {
      Serial.println("Client is connected!");

      sensors_event_t linearXL, gyro, magnetometer;
      double previousAcceleration = -1.0;

      uint8_t loadCount = 0;
      float movingAverageXL[MA_AMOUNT] = {};
      float previousAverage = -1.0;
      bool icpMeasuring = false;
      uint16_t stopCount = STOP_DURATION;
      uint16_t startCount = START_DURATION;

      uint8_t currentXLBufferSize = IMU_BUFFER_SIZE;

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
      int count = 100;

      while (SerialBT.hasClient()) {
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
        cycleStart = millis();

        totalCount += 1;

        bno.getEvent(&linearXL, Adafruit_BNO055::VECTOR_LINEARACCEL);
        bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno.getEvent(&magnetometer, Adafruit_BNO055::VECTOR_MAGNETOMETER);

        // Units m/s^2
        float xl_x = linearXL.acceleration.x;
        float xl_y = linearXL.acceleration.y;
        float xl_z = linearXL.acceleration.z;

        float magnitude = sqrt(xl_x * xl_x + xl_y * xl_y + xl_z * xl_z);

        float average = shiftAndPop(movingAverageXL, MA_AMOUNT, magnitude);

//         Serial.print("Transmit State: ");
//         Serial.print(transmitState);
//         Serial.print(" The difference: ");
//         Serial.print(abs(average - previousAverage));
//         Serial.print(" The stop count: ");
//         Serial.println(stopCount);

        if (loadCount < MA_AMOUNT) {
          // Load up the buffers for the moving average
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
                  Serial.println("Starting transmission!");
                  transmitState = TRANSMIT;
                  start = millis();
                  totalCount = 0;

                  EventPacket startPacket = EventPacket {
                    0x01
                  };

                  xQueueSend(eventQueue,
                             (void*)&startPacket,
                             ( TickType_t ) 0);

                  for (uint16_t i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
                    xQueueSend(pressureQueue,
                               (void*)&pressureTempBuffer[i],
                               ( TickType_t ) 0);
                  }
                  for (uint16_t i = 0; i < IMU_BUFFER_SIZE; i++) {
                    xQueueSend(imuQueue,
                               (void*)&imuBuffer[i],
                               ( TickType_t ) 0);
                  }
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

                  EventPacket stopPacket = EventPacket {
                    0x02
                  };
                  xQueueSend(eventQueue, (void*)&stopPacket, (TickType_t) 0);

                  stopCount = 0;

                  Serial.println(millis() - start);
                  Serial.print("Elapsed time: ");
                  Serial.print("XL and Quat Count: ");
                  Serial.print(xlCountDebug);
                  Serial.print(" Pressure Count: ");
                  Serial.println(pCount);
                  start = 0;

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

        IMUPacket imuPacket = IMUPacket {
          linearXL.acceleration.x,
          linearXL.acceleration.y,
          linearXL.acceleration.z,
          gyro.acceleration.x,
          gyro.acceleration.y,
          gyro.acceleration.z,
          gyro.magnetic.x,
          gyro.magnetic.y,
          gyro.magnetic.z,
        };

        shiftLeft(imuBuffer, IMU_BUFFER_SIZE);
        imuBuffer[IMU_BUFFER_SIZE - 1] = imuPacket;

        if (transmitState == TRANSMIT) {
          xQueueSend(imuQueue,
                     (void*)&imuPacket,
                     ( TickType_t ) 0);
          xlCountDebug++;
        }
      }
    }

    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void clampCheckLoop(void* params) {
  delay(50);
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;

  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    closedState = digitalRead(CLAMP_PIN);
  }
}

/**
 * STARTUP LOGIC
 */
void setup() {
  Serial.begin(115200);

  vTaskStartScheduler();

  pressureQueue = xQueueCreate(600, sizeof(PressurePacket));
  imuQueue = xQueueCreate(600, sizeof(IMUPacket));
  eventQueue = xQueueCreate(10, sizeof(EventPacket));

  delay(500);

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
    1024 * 2 * 2,
    ( void* ) 1,
    tskIDLE_PRIORITY,
    &bluetoothHandle
  );

  Serial.println("Finished collar startup!");
}

// Main loop is empty. Logic should be running in tasks.
void loop() {}
