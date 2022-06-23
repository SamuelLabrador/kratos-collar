cmake_minimum_required(VERSION 3.21)
project(kratos_collar)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories("C:/Program Files (x86)/Arduino/hardware/arduino/avr/cores/arduino")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/ArduinoBLE/src")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Arduino_LSM6DSOX/src")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/WiFiNINA/src")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_DPS310")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_BusIO")
include_directories("C:/Users/Samuel/Documents/Arduino/libraries/Adafruit_Unified_Sensor")
include_directories("C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/Wire/src")
include_directories("C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SPI/src")
include_directories("C:/arduino")


add_executable(kratos_collar
        LSM6DSOX_CUSTOM.cpp
        kratos-collar.ino)
