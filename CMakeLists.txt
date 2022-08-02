cmake_minimum_required(VERSION 3.21)
project(kratos_collar)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories("C:/Program Files (x86)/Arduino/hardware/arduino/avr/cores/arduino")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/ArduinoBLE/src")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/ArduinoBLE/src/utility")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Arduino_LSM6DSOX/src")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/ArduinoJson/src")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_DPS310")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_BNO055")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_BusIO")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_Unified_Sensor")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_LC709203F")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/ArduinoSTL/src")
include_directories("C:/Users/Samuel/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.4/libraries/BluetoothSerial/src")
include_directories("C:/Users/Samuel/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.4/tools/sdk/esp32s3/opi_qspi/include")
include_directories("C:/Users/Samuel/AppData/Local/Arduino15/packages/esp32/hardware/esp32/2.0.4/tools/sdk/esp32s3/include/bt/host/bluedroid/api/include/api")
include_directories("C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/Wire/src")
include_directories("C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SPI/src")
include_directories("C:/arduino")

add_executable(kratos_collar
        icp101xx.cpp
        kratos-collar.ino)
