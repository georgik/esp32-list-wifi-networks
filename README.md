# List of WiFi networks for ESP32-S3-BOX

Simple application which lists available networks.

## Set up the build environment
idf.py set-target esp32s3

## Build the project
idf.py build

## Flash the project to your ESP32-S3
idf.py -p /dev/ttyUSB0 flash

## Monitor the serial output
idf.py monitor
