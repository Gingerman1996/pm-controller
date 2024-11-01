# PM2.5 Room Control System

This project is designed to create an environment for testing PM2.5 measurement devices by maintaining the PM2.5 concentration in a room at a specified level. The system uses either ESP-NOW or HTTP communication between ESP32 devices, depending on the configuration, to monitor and adjust the PM2.5 levels by controlling fans or air purifiers.

## Overview

The system monitors PM2.5 levels and adjusts fan speeds to maintain the air quality in a room. It consists of several components implemented in different C++ files, each responsible for specific functionalities, such as data logging, PM sensor communication, and wireless control using either ESP-NOW or HTTP.

## Features

- **PM2.5 Monitoring**: Continuously measures the PM2.5 level in the room using a PMS sensor.
- **Automatic Control**: Adjusts the fan speed to maintain the PM2.5 level within the desired range.
- **ESP-NOW Communication**: Uses the ESP-NOW protocol for communication between master and slave ESP32 devices, allowing efficient and fast data transfer.
- **HTTP Communication**: Retrieves data from a server via API to adjust control settings or get external air quality information.
- **Logging**: Logs system events and PM2.5 measurements for analysis.

## File Descriptions

- **Calculator.cpp / Calculator.h**: Contains the logic for calculating the appropriate fan speed based on the current PM2.5 level and the target level.
- **ESPNowMaster.cpp / ESPNowMaster.h**: Manages communication between ESP32 devices using ESP-NOW, acting as the master node.
- **MyLog.cpp / MyLog.h**: Provides functionality for logging system events, errors, and PM2.5 levels to help with monitoring and debugging.
- **PMS.cpp / PMS.h**: Implements the interface to the PM2.5 sensor, handling data acquisition from the sensor and providing the current PM2.5 levels.
- **main.cpp**: The main entry point of the application. It initializes the components, manages the control loop, and coordinates interactions between different modules, including configuring ESP-NOW or HTTP communication.

## How to Use

### Hardware Setup

- Connect the PM2.5 sensor (e.g., PMS5003) to one of the ESP32 devices.
- Connect fans or air purifiers to a relay module, which will be controlled by the ESP32.
- Ensure that all ESP32 devices are on the same Wi-Fi channel to use ESP-NOW effectively.

### Software Setup

- Install [PlatformIO](https://platformio.org/) in Visual Studio Code.
- Install the required libraries for ESP-NOW, PMS sensors, and other dependencies.
- Upload the code to the ESP32 devices.

### Configuration

- Modify `main.cpp` to set the target PM2.5 level that you want to maintain in the room.
- Adjust the communication settings in `ESPNowMaster.cpp` to match your network setup if necessary.
- Configure whether the system should use ESP-NOW or HTTP communication by setting the appropriate parameters in `main.cpp`.

### Running the System

- Power on the ESP32 devices.
- The master ESP32 will read the PM2.5 data from the sensor and communicate with the slave devices to control the fan speed.
- If configured for HTTP, the system will retrieve data from the server via API as needed.
- The system will automatically adjust the fan to maintain the desired PM2.5 level.

## Dependencies

- **ESP-NOW Library**: For wireless communication between ESP32 devices.
- **PMS Sensor Library**: To interface with the PM2.5 sensor.
- **Arduino Core for ESP32**: For basic functionality of ESP32 microcontrollers.
- **HTTP Client Library**: For communicating with external servers via HTTP.

## Future Improvements

- **Mobile App Integration**: Add a mobile app interface to allow users to set the PM2.5 level and monitor air quality in real time.
- **Machine Learning**: Use machine learning to predict PM2.5 trends and adjust fan speed proactively.
- **Additional Sensors**: Integrate other air quality sensors (e.g., CO2, temperature, humidity) for a more comprehensive indoor environment control.

## Contributing

Contributions are welcome! Please feel free to open issues or submit pull requests for improvements or bug fixes.

