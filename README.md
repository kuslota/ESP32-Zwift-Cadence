# ESP32-Zwift-Cadence

## Overview

This project is designed to use an ESP32 to detect the cadence of a bike trainer using a Hall effect sensor and transmit the data over BLE to applications like Zwift.

## Features

- Uses BLE (Bluetooth Low Energy) to transmit cadence data.
- Utilizes the Cycling Speed and Cadence (CSC) profile of BLE.
- Debounces the Hall effect sensor to avoid false readings.
- Compatible with the ESP32 development board.

## Getting Started

### Prerequisites
- ESP32 development board.
- A3144 Hall effect sensor.
- PlatformIO IDE or similar environment for ESP32 development.

### Setup
1. Clone the repository: `git clone https://github.com/kuslota/ESP32-Zwift-Cadence.git`
2. Open the project in PlatformIO or your preferred IDE.
3. Ensure you have the necessary libraries installed. The main libraries used are:
- Arduino.h
- BLEDevice.h
- BLEUtils.h
- BLEServer.h
- BLE2902.h

### Configuration
The platformio.ini file contains the configuration for the ESP32 board. The current settings are:

- Platform: espressif32@3.5.0
- Board: esp32dev
- Framework: arduino
- Monitor Speed: 115200
- Upload Speed: 115200

### Usage
1. Connect the Hall effect sensor to pin 32 of the ESP32.
2. Upload the code to the ESP32.
3. Start pedaling on your bike trainer. The ESP32 will detect the cadence and transmit the data over BLE.

## Contributing

Feel free to open issues or pull requests if you wish to contribute to this project.
