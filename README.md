# Motion Detection and Alert System Using ESP32

## Overview

This project implements a motion detection and alert system using the ESP32 microcontroller. By integrating Passive Infrared (PIR) and ultrasonic sensors, the system provides reliable motion detection with reduced false positives. It sends real-time alerts to a designated Telegram account using the Telegram Bot API and includes features such as event logging, sensor health monitoring, and intelligent power management through sleep modes.

## Features

- **Dual-Sensor Verification**: Combines PIR and ultrasonic sensors to accurately detect and confirm motion events.
- **Real-Time Alerts**: Sends instant notifications via Telegram when motion is detected and confirmed.
- **Event Logging**: Records motion detection events with timestamps, distances, and sensor statuses using non-volatile storage.
- **Sensor Health Monitoring**: Regularly checks the functionality of the PIR sensor and adapts operation if a malfunction is detected.
- **Power Management**: Utilizes the ESP32's light sleep mode to reduce power consumption while maintaining responsiveness.
- **Remote Control via Telegram**: Supports commands to retrieve logs, clear logs, and interact with the system remotely.

## Hardware Requirements

- **ESP32 Development Board**: [ESP32-WROOM-32](https://www.espressif.com/en/products/modules/esp32)
- **PIR Motion Sensor**: HC-SR501 or equivalent
- **Ultrasonic Sensor**: HC-SR04
- **Active Buzzer**: For audible alerts
- **LED Indicator**: For visual alerts
- **Jumper Wires and Breadboard**: For connections
- **Power Supply**: USB cable or appropriate battery (ensure adequate voltage and current)

## Software Requirements

- **Arduino IDE**: Version 1.8 or higher
- **ESP32 Board Package**: Install via Arduino IDE Boards Manager
- **Libraries**:
  - [WiFi](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi) (Included with ESP32 Board Package)
  - [WiFiClientSecure](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFiClientSecure) (Included with ESP32 Board Package)
  - [UniversalTelegramBot](https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot) (Install via Library Manager)
  - [ArduinoJson](https://github.com/bblanchon/ArduinoJson) (Version 6.x or higher, install via Library Manager)
  - [Preferences](https://github.com/espressif/arduino-esp32) (Included with ESP32 Board Package)
  - ESP32 Core Libraries:
    - esp_sleep.h (Included with ESP32 Board Package)
    - driver/rtc_io.h (Included with ESP32 Board Package)

### Configuration Requirements

- WiFi network credentials
- Telegram Bot Token (obtain from BotFather)
- Telegram Chat ID
- Valid GPIO pin assignments for sensors

### Development Environment Recommendations

- Use latest stable Arduino IDE version
- Enable verbose output during upload (Tools > Upload Speed: "Verbose")
- Configure appropriate upload speed (typically 115200)
- Select correct COM port for your ESP32 device
