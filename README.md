# Gate Counter Project

## Overview
The Gate Counter project is designed to monitor vehicle traffic exiting a park using sensors and microcontroller technology. It uses an ESP32 microcontroller to:

- Detect vehicles using a magnetic sensor and beam sensor.
- Log and publish vehicle counts via MQTT.
- Save data to an SD card for persistence and analysis.
- Provide real-time statistics via an OLED display.
- Enable remote monitoring and updates using WiFi and Over-The-Air (OTA) updates.

This system is an essential tool for traffic control and capacity management in parks or similar facilities.

## Features

### Sensors and Detection
- **Magnetic Sensor**: Detects metallic objects.
- **Beam Sensor**: Confirms vehicle presence.
- **Car Detection Logic**: Uses a state machine for accurate detection.

### Data Management
- Saves hourly, daily, and show-specific vehicle counts to an SD card.
- Provides recovery and persistence across reboots.
- Publishes data via MQTT to a remote server.

### Networking
- Supports multiple WiFi networks using WiFiMulti.
- Publishes car counts, temperature, humidity, and other metrics via MQTT.
- Includes a web server for file operations and remote interactions.

### Real-Time Display
- OLED display provides live updates on vehicle counts, temperature, and other metrics.

### Environmental Monitoring
- Logs temperature and humidity data using a DHT22 sensor.

### Timekeeping
- Synchronizes time using an RTC and NTP server.
- Automates actions such as resetting counts and saving summaries at specific times.

### OTA Updates
- ElegantOTA integration allows for seamless firmware updates.

---

## Documentation

For more detailed information about specific components, see the following:

- [MQTT Reset Topics](docs/mqtt_reset_topics.md): Detailed descriptions of MQTT reset topics and their payloads.
- [Changelog](docs/changelog.md): A history of updates, fixes, and new features in the Gate Counter project.

---

## Setup Instructions
1. **Hardware Requirements**:
   - ESP32 microcontroller (DOIT DevKit V1 recommended).
   - DHT22 sensor for temperature and humidity.
   - Magnetic and beam sensors for vehicle detection.
   - OLED display (128x64 resolution).
   - SD card and reader module.

2. **Software Requirements**:
   - Arduino IDE with necessary libraries:
     - `ArduinoJson`
     - `PubSubClient`
     - `Adafruit_GFX` and `Adafruit_SSD1306`
     - `RTClib`, `NTPClient`
     - `DHT` and `DHT_U`

3. **Connections**:
   - Connect sensors, SD card, and OLED display to the ESP32 as specified in the code comments.

4. **Configuration**:
   - Edit `secrets.h` to include WiFi credentials.
   - Adjust MQTT topics and server settings as needed.

5. **Deployment**:
   - Upload the firmware using Arduino IDE.
   - Monitor system logs via Serial Monitor for debugging.

---

## Future Enhancements
- Integrate a web-based dashboard for real-time monitoring.
- Add support for additional sensors and analytics.
- Improve power efficiency for extended deployments.

---

For further assistance or to contribute to the project, please contact the developer:
**Greg Liebig**  
Email: `gliebig@sheboyganlights.org`

