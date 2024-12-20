# MQTT Reset Topics - Gate Counter

This document provides descriptions for the MQTT reset topics supported by the Gate Counter system. These topics allow remote control and configuration of various counters and states within the system.

---

## Subscribed Topics for Resets

### 1. `msb/traffic/GateCounter/resetDailyCount`
- **Description**: Resets the total daily vehicle count to a specified value.
- **Payload**: Integer representing the new value for the daily counter.
- **Example**:
  - Payload: `0` (Resets the daily vehicle count to 0.)

---

### 2. `msb/traffic/GateCounter/resetShowCount`
- **Description**: Resets the total show vehicle count.
- **Payload**: Integer representing the new value for the show counter.
- **Example**:
  - Payload: `50` (Sets the show vehicle count to 50.)

---

### 3. `msb/traffic/GateCounter/resetDayOfMonth`
- **Description**: Manually resets the calendar day to a specific value.
- **Payload**: Integer representing the new day of the month.
- **Example**:
  - Payload: `15` (Sets the day of the month to the 15th.)

---

### 4. `msb/traffic/GateCounter/resetDaysRunning`
- **Description**: Resets the total number of days the show has been running.
- **Payload**: Integer representing the new value for days running.
- **Example**:
  - Payload: `10` (Resets the "days running" counter to 10.)

---

### 5. `msb/traffic/GateCounter/gateCounterTimeout`
- **Description**: Updates the timeout duration for the gate counter.
- **Payload**: Integer in milliseconds specifying the new timeout.
- **Example**:
  - Payload: `60000` (Sets the timeout duration to 60 seconds.)

---

### 6. `msb/traffic/GateCounter/carDetectMS`
- **Description**: Updates the minimum wait duration for confirming a vehicle.
- **Payload**: Integer in milliseconds specifying the new detection duration.
- **Example**:
  - Payload: `1500` (Sets the detection duration to 1.5 seconds.)

---

### 7. `msb/traffic/GateCounter/loggingEnabled`
- **Description**: Toggles the logging of sensor data.
- **Payload**:
  - `1`: Enables logging.
  - `0`: Disables logging.
- **Example**:
  - Payload: `1` (Enables logging of sensor data.)

---

## Notes
- Ensure all payloads are sent as valid integers where applicable.
- Use these topics to remotely manage the Gate Counter system for improved flexibility and responsiveness.

---

For further assistance, contact the system administrator or refer to the Gate Counter documentation.

