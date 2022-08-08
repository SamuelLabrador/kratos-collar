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

#define CLAMP_PWR 14
#define CLAMP_PIN 13
#define YELLOW_LED 19
#define RED_LED 15

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

struct SensorPacket {
  // [xl_x, xl_y, xl_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z, pressure, temp]
  float data[11];
};

struct EventPacket {
  unsigned char event;
};

/**
 * GLOBAL VARIABLES
 */
const uint16_t SENSOR_FREQUENCY = 66;
bool closedState = false;
XL_TRANSMIT_STATE transmitState = WAIT_TRANSMIT;
const uint16_t STOP_DURATION = SENSOR_FREQUENCY * 3;
const uint16_t START_DURATION = 15;

// These buffers are used to save the last ~1 second of data.
// These are transmitted when motion is detected.
const uint16_t IMU_BUFFER_SIZE = 100;
//const uint16_t PRESSURE_BUFFER_SIZE = 40;
SensorPacket imuBuffer[IMU_BUFFER_SIZE];
//PressurePacket pressureTempBuffer[PRESSURE_BUFFER_SIZE];

// DEBUG VARIABLES
unsigned long pCount = 0;

// Queues are used to queue packets to be transmitted by the bluetooth task.
QueueHandle_t sensorQueue;
QueueHandle_t eventQueue;

TaskHandle_t xHandle = NULL;
TaskHandle_t clampHandle = NULL;
TaskHandle_t bluetoothHandle = NULL;
TaskHandle_t ledHandle = NULL;

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

void shiftLeft(SensorPacket *buffer, uint16_t size) {
  for (uint8_t i = 0; i < size - 1; i++) {
    buffer[i] = buffer[i + 1];
  }
}

/**
 * CONFIGURATION FUNCTIONS
 */
void configureBarometer() {
   Serial.println("Configuring barometer");
  if (!icp.begin()) {
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
  delay(1000);
  for (;;) {
    SensorPacket sensorPacket;
    EventPacket eventPacket;

    if (sensorQueue != NULL && eventQueue != NULL) {
      String payload ="";
      unsigned int count = 0;
      const unsigned int limit = 15; // Read AT MOST 15 packets from each queue

      if (xQueueReceive(eventQueue, &(eventPacket), (TickType_t) 0) == pdPASS) {
        StaticJsonDocument<256> eventDoc;
        eventDoc["e"] = "E";
        eventDoc["d"] = eventPacket.event;

        serializeJson(eventDoc, payload);
      }

      count = 0;
      while (xQueueReceive(sensorQueue, &(sensorPacket), (TickType_t) 0) == pdPASS) {
        DynamicJsonDocument imuDoc(2048);
        imuDoc["e"] = "I";
        for (uint8_t i = 0; i < 11; i++) {
          imuDoc["d"][i] = sensorPacket.data[i];
        }
        serializeJson(imuDoc, payload);

        count++;
        if (count > limit) {
          break;
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

void mainTask(void* params) {
  delay(1000);
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

      // Scheduling variables
      TickType_t xLastWakeTime;
      BaseType_t xWasDelayed;
      const TickType_t xFrequency = 15 / portTICK_PERIOD_MS;

      icp.measureStart(ICP101xx::NORMAL);
      xLastWakeTime = xTaskGetTickCount();

      while (SerialBT.hasClient()) {
        xWasDelayed = xTaskDelayUntil( &xLastWakeTime, xFrequency );

        if (xWasDelayed == pdFALSE) {
          Serial.println("THE IMU TASK RAN LATE");
        }

        if (!icp.dataReady()) {
          Serial.println("The ICP DATA IS NOT READY!");
        }

        totalCount += 1;

        // Kick off measurements. VERY IMPORTANT. This must go before getting the IMU data
        float pressure = icp.getPressurePa();
        float temperature = icp.getTemperatureC();
        icp.measureStart(ICP101xx::NORMAL);

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

                  for (uint16_t i = 0; i < IMU_BUFFER_SIZE; i++) {
                    xQueueSend(sensorQueue,
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

                  unsigned long duration = millis() - start;
                  Serial.print("Elapsed time: ");
                  Serial.print(duration);
                  Serial.print(" Value Count: ");
                  Serial.println(xlCountDebug);
                  start = 0;

                  xlCountDebug = 0;
                  pCount = 0;
                }
                break;
            }
          } else if (!closedState && transmitState == TRANSMIT) {
            Serial.println("Stopping transmission!");
            transmitState = WAIT_TRANSMIT;

            EventPacket stopPacket = EventPacket {
              0x02
            };
            xQueueSend(eventQueue, (void*)&stopPacket, (TickType_t) 0);

            stopCount = 0;

            unsigned long duration = millis() - start;
            Serial.print("Elapsed time: ");
            Serial.print(duration);
            Serial.print(" Value Count: ");
            Serial.println(xlCountDebug);
            start = 0;

            xlCountDebug = 0;
            pCount = 0;
          }
          previousAverage = average;
        }

        // Build XL Packet
        SensorPacket imuPacket = SensorPacket {
          xl_x,
          xl_y,
          xl_z,
          gyro.acceleration.x,
          gyro.acceleration.y,
          gyro.acceleration.z,
          magnetometer.magnetic.x,
          magnetometer.magnetic.y,
          magnetometer.magnetic.z,
          pressure,
          temperature
        };

        shiftLeft(imuBuffer, IMU_BUFFER_SIZE);
        imuBuffer[IMU_BUFFER_SIZE - 1] = imuPacket;

        if (transmitState == TRANSMIT) {
          xQueueSend(sensorQueue,
                     (void*)&imuPacket,
                     ( TickType_t ) 0);
          xlCountDebug++;
        }

//        unsigned duration = millis() - cycleStart;
//        if (duration > 10) {
//          Serial.print("WARNING! CYCLE WENT OVER 10ms! Duration: ");
//          Serial.println(duration);
//        }
      }
    }

    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void clampCheckLoop(void* params) {
  delay(1000);
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;

  for (;;) {
    vTaskDelay(xFrequency);
    closedState = digitalRead(CLAMP_PIN);
  }
}

void ledTask(void* params) {
  for (;;) {
    if (SerialBT.hasClient()) {
      digitalWrite(YELLOW_LED, true);
      digitalWrite(RED_LED, transmitState == TRANSMIT);
      vTaskDelay(200 / portTICK_PERIOD_MS);

      digitalWrite(YELLOW_LED, closedState);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    } else {
      digitalWrite(YELLOW_LED, false);
      digitalWrite(RED_LED, false);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
  }
}

void debugTask(void* params) {
  for (;;) {
    if (SerialBT.hasClient()) {
//      Serial.print("Last measured pressure: ");
//      Serial.println(pressureTempBuffer[PRESSURE_BUFFER_SIZE - 1].data[0]);

      delay(5000);
    }
  }
}

/**
 * STARTUP LOGIC
 */
void setup() {
  Serial.begin(115200);

  sensorQueue = xQueueCreate(1024, sizeof(SensorPacket));
  eventQueue = xQueueCreate(512, sizeof(EventPacket));

  Serial.println("Collar warming up...");

  configureBluetooth();
  configureAccel();
  configureBarometer();

  pinMode(CLAMP_PIN, INPUT);
  pinMode(CLAMP_PWR, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(CLAMP_PWR, HIGH);

  transmitState = WAIT_TRANSMIT;

  BaseType_t xReturned;

  // Accelerometer task
  xReturned = xTaskCreate(
    mainTask,        /* Function that implements the task. */
    "MAIN_TASK",          /* Text name for the task. */
    1024 * 2,      /* Stack size in words, not bytes. */
    ( void * ) 1,    /* Parameter passed into the task. */
    10,         /* Priority at which the task is created. */
    &xHandle );      /* Used to pass out the created task's handle. */

  if (xReturned != pdPASS) {
    Serial.println("UNABLE TO CREATE MAIN TASK");
    while(1);
  }

  // Clamp detection Task
  xReturned = xTaskCreate(
    clampCheckLoop,
    "CLAMP_TASK",
    1024,
    ( void* ) 2,
    tskIDLE_PRIORITY,
    &clampHandle
  );

  if (xReturned != pdPASS) {
    Serial.println("UNABLE TO CREATE CLAMP TASK");
    while(1);
  }

  // Bluetooth task
  xReturned = xTaskCreate(
    bluetoothTransmitTask,
    "BLUETOOTH_TASK",
    1024 * 2 * 2,
    ( void* ) 1,
    3,
    &bluetoothHandle
  );

  if (xReturned != pdPASS) {
    Serial.println("UNABLE TO CREATE BLUETOOTH TASK");
    while(1);
  }

  // Debug task
  xReturned = xTaskCreate(
    ledTask,
    "LED_TASK",
    1024,
    ( void* ) 1,
    3,
    &ledHandle
  );

  if (xReturned != pdPASS) {
    Serial.println("UNABLE TO CREATE LED TASK");
    while(1);
  }

  Serial.println("Finished collar startup!");
}

// Main loop is empty. Logic should be running in tasks.
void loop() {}
