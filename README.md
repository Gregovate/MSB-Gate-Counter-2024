This is version 2 of the gate counter with the added beam sensor and a newly installed magnotometer. Added MQTT Server at park

#define topic_base_path "msb/traffic/GateCounter"
#define MQTT_PUB_TOPIC0 "msb/traffic/GateCounter/hello"
#define MQTT_PUB_TOPIC1 "msb/traffic/GateCounter/temp"
#define MQTT_PUB_TOPIC2 "msb/traffic/GateCounter/time"
#define MQTT_PUB_TOPIC3 "msb/traffic/GateCounter/ExitTotal"
#define MQTT_PUB_TOPIC4 "msb/traffic/GateCounter/inParkCars"
#define MQTT_PUB_TOPIC5 "msb/traffic/GateCounter/Exit_18"
#define MQTT_PUB_TOPIC6 "msb/traffic/GateCounter/Exit_19"
#define MQTT_PUB_TOPIC7 "msb/traffic/GateCounter/Exit_20"
#define MQTT_PUB_TOPIC8 "msb/traffic/GateCounter/Exit_21"
#define MQTT_PUB_TOPIC9 "msb/traffic/GateCounter/ExitTotal"
#define MQTT_PUB_TOPIC10 "msb/traffic/GateCounter/ShowTotal"
#define MQTT_PUB_TOPIC11 "msb/traffic/GateCounter/TTP"
#define MQTT_PUB_TOPIC12 "msb/traffic/GateCounter/beamSensorState"
#define MQTT_PUB_TOPIC13 "msb/traffic/GateCounter/magSensorState"
// Subscribing Topics (to reset values)
#define MQTT_SUB_TOPIC0  "msb/traffic/CarCounter/EnterTotal"
#define MQTT_SUB_TOPIC1  "msb/traffic/GateCounter/resetDailyCount"
#define MQTT_SUB_TOPIC2  "msb/traffic/GateCounter/resetShowCount"
#define MQTT_SUB_TOPIC3  "msb/traffic/GateCounter/resetDayOfMonth"
#define MQTT_SUB_TOPIC4  "msb/traffic/GateCounter/resetDaysRunning"
#define MQTT_SUB_TOPIC5  "msb/traffic/GateCounter/gateCounterTimeout"
#define MQTT_SUB_TOPIC6  "msb/traffic/GateCounter/resetCalendarDay"