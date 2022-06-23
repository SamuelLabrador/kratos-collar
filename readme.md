## To Compile

arduino-cli.exe compile --fqbn arduino:mbed_nano:nanorp2040connect .\kratos-collar.ino

## To Upload

arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:samd:mkr1000 .\kratos-collar.ino
