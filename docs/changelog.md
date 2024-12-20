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

