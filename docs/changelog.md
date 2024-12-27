### [24.12.26.1] - YYYY-MM-DD
- Added alarm if beam sensor remains high for more than 3 minutes

### [24.12.19.6] - YYYY-MM-DD
- Removed temperature array from average procedure an used the global declared array

### [24.12.19.5] - YYYY-MM-DD
- Accidentally removed mqtt_client.setCallback(callback) Fixed with a forward declaration

### [24.12.19.4] - YYYY-MM-DD
- Added save saveHourlyCounts() to countTheCar() and removed save from OTA update

### [24.12.18.4] - YYYY-MM-DD
- added  pinMode(DHTPIN, INPUT_PULLUP) for DHT Sensor getting bad readings.  77.1% (used 1010645 bytes from 1310720 bytes)

### [24.12.18.2] - YYYY-MM-DD
- put publishing state changes beamSensorState and magSensorState and timeToPassMS. detectCar() finally working reliably!

### [24.12.18.1] - YYYY-MM-DD
- Renamed hourlyCarCount[] to hourlyCount[] and finished comparison to Car Counter Code

### [24.12.17.4] - YYYY-MM-DD
- added new topic MQTT_COUNTER_LOG "msb/traffic/GateCounter/CounterLog"

### [24.12.17.3] - YYYY-MM-DD
- more tweaks to detectCar() revised averageHourlyTemp() & readTempandRH() removed averageHourlyTemp() from Loop

### [24.12.17.2] - YYYY-MM-DD
- modified detectCar() based on logging data increased predetect mag sensor to 750ms

### [24.12.17.1] - YYYY-MM-DD
- modified downloadSDFile(AsyncWebServerRequest *request) to save file with correct filename

### [24.12.16.4] - YYYY-MM-DD
- Reset counts was incorrect due to syncing code from car counter to gate counter should be 5:10 pm for gate counter

### [24.12.16.3] - YYYY-MM-DD
- was not incrementing hourly car counts and writing multiple rows for the same date

### [24.12.16.2] - YYYY-MM-DD
- removed all references to tempF from RTC sensor moved to DHT22 sensor publish every 10 min publish temp & RH json format

### [24.12.16.1] - YYYY-MM-DD
- revised saveDailyShowSummary() times to 9:20 for gate counter

### [24.12.15.4] - YYYY-MM-DD
- revised saveDailyShowSummary() to average temps during show

### [24.12.15.3] - YYYY-MM-DD
- after debugging in field after 15.2

### [24.12.15.2] - YYYY-MM-DD
- Matched Car Counter procedures, Changed Names, Added webserver

### [24.12.14.2] - YYYY-MM-DD
- added getdayofmonth() after updatingdayofmonth()

### [24.12.14.1] - YYYY-MM-DD
- Added DHT22 sensor to gate counter. Revised carDetect to weight beamSensor higher and used magSensor for additional Confirmation

### [24.12.12.3] - YYYY-MM-DD
- Tweaks to state machine for improving accuracy. Added timers for analysis topics 16 & 18

### [24.12.12.2] - YYYY-MM-DD
- Revising detectCars() State machine logic for new sensor

### [24.12.12.1] - YYYY-MM-DD
- BREAKING CHANGE Replaced reflective sensor with through beam nomally closed invert reading

### [24.12.02.3] - YYYY-MM-DD
- added timer for timeDiff between magSensor HIGH and beamSensor HIGH and timer between cars

### [24.12.02.1] - YYYY-MM-DD
- revised state machine again for through-beam sensor and removed bounce detection for beam

### [24.12.01.3] - YYYY-MM-DD
- revised state machine again for beam sensor bouncing during car detection

### [24.12.01.2] - YYYY-MM-DD
- revised state machine for beam sensor bouncing

### [24.12.01.1] - YYYY-MM-DD
- converted car dection to state machine in separate branch

### [24.11.30.3] - YYYY-MM-DD
- Totally Reworked Car Detection

### [24.11.30.2] - YYYY-MM-DD
- Refactored publishMQTT and File Checking and Creation and for loop hour counts

### [24.11.30.1] - YYYY-MM-DD
- Updated MQTT callback section to clean up potential memory leak

### [24.11.27.1] - YYYY-MM-DD
- Changed mqtt publish outside File writes, changed daily totals to string pointer, tempF to float

### [24.11.26.1] - YYYY-MM-DD
- Changed Update to days running since they were doubling on date change

### [24.11.25.2] - YYYY-MM-DD
- Added in state change in loop to publish mag sensor states. Missed 5 cars during dog show.

### [24.11.25.1] - YYYY-MM-DD
- Added MQTT Topics for Remote Reset to match Car Counter. Added Alarm for blocked beam sensor

### [24.11.19.1] - YYYY-MM-DD
- Replace Current Day & Calday with DayOfMonth. Added boolean to print daily summary once

### [24.11.18.1] - YYYY-MM-DD
- Added publishing totals on manual reset

### [24.11.15.1] - YYYY-MM-DD
- Removed Sensor Bounces, updated MQTT Topics

### [24.11.14.1] - YYYY-MM-DD
- Eliminated mqtt timeout, debug topic, added TTP to mqtt

### [24.11.10.2] - YYYY-MM-DD
- Miscellaneous formatting issues before re-creating JSON branch again

### [24.11.10.1] - YYYY-MM-DD
- Fixed bounce check, changed filename methods merged Arduino json branch

### [24.10.28.1] - YYYY-MM-DD
- Created proceedure for Updating Car Counts

### [24.10.27.1] - YYYY-MM-DD
- simplified car detect logic, Formatting Changes

### [24.10.23.3] - YYYY-MM-DD
- Fixed File creation errors

### [24.10.23.2] - YYYY-MM-DD
- Added update/reset check in loop for date changes. Created initSDCard().

### [24.10.23.1] - YYYY-MM-DD
- Updated totals, bug fixes, files ops comparrison to Gate counter  added file ops

### [24.10.17.2] - YYYY-MM-DD
- added #define FWVersion

### [24.10.15.0] - YYYY-MM-DD
- Fixed Pin problem. Beam & mag sensor swapped causing the problems. Purpose: suppliments Car Counter to improve traffic control and determine park capacity

### [23.12.13.0] - YYYY-MM-DD
- Changed time format YYYY-MM-DD hh:mm:ss 12/13/23

# Changelog

## Version History

### Version 24.12.19.3
- Synced shared code between Gate Counter and Car Counter.
- Resolved warnings and removed unused variables.
- Updated `carDetectMS` default value to 1200ms.
- Enhanced SD card file operations for better persistence.
- Fixed remaining issues with HTML file handling.

### Version 24.12.19.2
- Addressed warnings during compilation.
- Improved handling of unused variables.
- Simplified configuration settings.

### Version 24.12.19.1
- Added functionality to save hourly data before firmware updates.
- Set hostname dynamically for multiple instances.
- Improved MQTT topic structure for consistency.
- Enhanced firmware update compatibility by modifying `secrets.h`.

### Version 24.12.18.6
- Introduced `flagHourlyReset` for better hourly data saving.
- Enhanced logging capabilities for analysis and debugging.

### Version 24.12.18.5
- Added dynamic MQTT topics for hourly car totals.
- Enhanced reliability of `detectCar()` functionality.

### Version 24.12.18.3
- Added new statistics for "Time Between Cars" and beam sensor high time.
- Refined state machine logic for car detection.

### Version 24.12.16.5
- Introduced logging functions for sensor data analysis.
- Updated daily reset logic for improved consistency.

### Version 24.11.25.3
- Improved MQTT topic management.
- Resolved car detection discrepancies during specific scenarios.

### Version 24.10.24.1
- Addressed errors in MQTT variable declarations.
- Refined daily total management for better accuracy.

---

## Notes
- For further details, refer to the documentation or contact the system administrator.
- Future releases will focus on enhancing real-time analytics and integrating additional sensor support.

---

For more information, see the [README.md](README.md) file.

