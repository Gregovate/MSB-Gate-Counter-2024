/*
Gate Counter by Greg Liebig gliebig@sheboyganlights.org
Initial Build 12/5/2023 12:15 pm
Counts vehicles as they exit the park
Connects to WiFi and updates RTC on Boot
Uses an Optocoupler to read burried vehicle sensor for Ghost Controls Gate operating at 12V
Purpose: suppliments Car Counter to improve traffic control and determine park capacity
Uses an Optocoupler to read burried vehicle sensor for Ghost Controls Gate operating at 12V
DOIT DevKit V1 ESP32 with built-in WiFi & Bluetooth

## BEGIN CHANGELOG ##
24.12.26.1 Added alarm if beam sensor remains high for more than 3 minutes
24.12.19.6 Removed temperature array from average procedure an used the global declared array
24.12.19.5 Accidentally removed mqtt_client.setCallback(callback) Fixed with a forward declaration
24.12.19.4 Added save saveHourlyCounts() to countTheCar() and removed save from OTA update
24.12.19.3 File comparrison between Car Counter and Gate Counter. Synced shared code
24.12.19.2 fixed warnings, removed unused variables, Changed waitduration to carDetectMS Only errors are with html. Changed carDetectMS default 1200
24.12.19.1 Included saving hourly data before uploading new firmware, Set hostname, MultiMQTT, modified secrets.h
24.12.18.6 Changed flags to save certain data hourly flagHourlyReset
24.12.18.5 Fixed MQTT dymanic topic in CountTheCar for hourly totals 
24.12.18.4 added  pinMode(DHTPIN, INPUT_PULLUP) for DHT Sensor getting bad readings.  77.1% (used 1010645 bytes from 1310720 bytes)
24.12.18.3 Added new stat Time Between Cars. Put back BeamSensor High Time
24.12.18.2 put publishing state changes beamSensorState and magSensorState and timeToPassMS. detectCar() finally working reliably!
24.12.18.1 Renamed hourlyCarCount[] to hourlyCount[] and finished comparison to Car Counter Code
24.12.17.4 added new topic MQTT_COUNTER_LOG "msb/traffic/GateCounter/CounterLog"
24.12.17.3 more tweaks to detectCar() revised averageHourlyTemp() & readTempandRH() removed averageHourlyTemp() from Loop
24.12.17.2 modified detectCar() based on logging data increased predetect mag sensor to 750ms
24.12.17.1 modified downloadSDFile(AsyncWebServerRequest *request) to save file with correct filename
24.12.16.5 Added logging function to plot sensor data. Changes getHourlyData() & saveHoulyCounts()
24.12.16.4 Reset counts was incorrect due to syncing code from car counter to gate counter should be 5:10 pm for gate counter
24.12.16.3 was not incrementing hourly car counts and writing multiple rows for the same date
24.12.16.2 removed all references to tempF from RTC sensor moved to DHT22 sensor publish every 10 min publish temp & RH json format
24.12.16.1 revised saveDailyShowSummary() times to 9:20 for gate counter
24.12.15.4 revised saveDailyShowSummary() to average temps during show
24.12.15.3 after debugging in field after 15.2
24.12.15.2 Matched Car Counter procedures, Changed Names, Added webserver
24.12.14.2 added getdayofmonth() after updatingdayofmonth()
24.12.14.1 Added DHT22 sensor to gate counter. Revised carDetect to weight beamSensor higher and used magSensor for additional Confirmation
24.12.12.3 Tweaks to state machine for improving accuracy. Added timers for analysis topics 16 & 18
24.12.12.2 Revising detectCars() State machine logic for new sensor
24.12.12.1 BREAKING CHANGE Replaced reflective sensor with through beam nomally closed invert reading
24.12.02.3 added timer for timeDiff between magSensor HIGH and beamSensor HIGH and timer between cars
24.12.02.1 revised state machine again for through-beam sensor and removed bounce detection for beam
24.12.01.3 revised state machine again for beam sensor bouncing during car detection
24.12.01.2 revised state machine for beam sensor bouncing
24.12.01.1 converted car dection to state machine in separate branch
24.11.30.3 Totally Reworked Car Detection
24.11.30.2 Refactored publishMQTT and File Checking and Creation and for loop hour counts
24.11.30.1 Updated MQTT callback section to clean up potential memory leak
24.11.27.1 Changed mqtt publish outside File writes, changed daily totals to string pointer, tempF to float
24.11.26.1 Changed Update to days running since they were doubling on date change
24.11.25.3 Cleaned up MQTT Topics
24.11.25.2 Added in state change in loop to publish mag sensor states. Missed 5 cars during dog show.
24.11.25.1 Added MQTT Topics for Remote Reset to match Car Counter. Added Alarm for blocked beam sensor
24.11.19.1 Replace Current Day & Calday with DayOfMonth. Added boolean to print daily summary once
24.11.18.1 Added publishing totals on manual reset
24.11.15.1 Removed Sensor Bounces, updated MQTT Topics
24.11.14.1 Eliminated mqtt timeout, debug topic, added TTP to mqtt
24.11.10.2 Miscellaneous formatting issues before re-creating JSON branch again
24.11.10.1 Fixed bounce check, changed filename methods merged Arduino json branch
24.11.9.2 Added mqtt publish when car counter cars updates
24.11.9.1 Added mqtt loop to while loop. Working code excluding elegantota update
24.11.8.1 testing beamSensorBoune time
24.11.6.2 Increased carDetectTime from 500 to 750 millis Sensor bounces with my truck
24.11.6.1 Changed MQTT Topics for GateCounter rather than exit
24.11.5.1 Fixed wrong publishing topic for carCounter Counts
24.11.4.1 Removed Bounce Logic for Beam Sensor and associated vatiables added logic for show totals
24.10.28.1 Created proceedure for Updating Car Counts
24.10.27.1 simplified car detect logic, Formatting Changes
24.10.24.1 Fixed Errors in MQTT variables
24.10.23.3 Fixed File creation errors
24.10.23.2 Added update/reset check in loop for date changes. Created initSDCard(). 
24.10.23.1 Updated totals, bug fixes, files ops comparrison to Gate counter  added file ops
24.10.17.2 added #define FWVersion
24.10.15.0 Fixed Pin problem. Beam & mag sensor swapped causing the problems. Purpose: suppliments Car Counter to improve traffic control and determine park capacity
23.12.13.0 Changed time format YYYY-MM-DD hh:mm:ss 12/13/23
## END CHANGELOG ## 
*/

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include "NTPClient.h"
//#include <WiFiClientSecure.h>
#include <WiFiMulti.h>
#include "secrets.h"
#include "time.h"
#include "SD.h"
#include "FS.h"
#include "SPI.h"
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#endif
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTAPro.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <queue>  // Include queue for storing messages

// ******************** CONSTANTS *******************
#define FWVersion "24.12.26.1"   // Firmware Version
#define OTA_Title "Gate Counter" // OTA Title
#define magSensorPin 32 // Pin for Magnotometer Sensor
#define beamSensorPin 33  //Pin for Reflective Beam Sensor
#define DHTPIN 4       // GPIO pin for the DHT22
#define DHTTYPE DHT22  // DHT TYPE
#define PIN_SPI_CS 5   // The ESP32 pin GPIO5
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// #define MQTT_KEEPALIVE 30 //removed 10/16/24

const int showStartMin = 17 * 60 + 10; // 5:10 PM in minutes
const int showEndMin = 21 * 60 + 20;   // 9:20 PM in minutes (including additional checks till 9:20 PM)
// **************************************************

/***** MQTT TOPIC DEFINITIONS *****/
#define THIS_MQTT_CLIENT "espGateCounter" // Look at line 90 and set variable for WiFi Client secure & PubSubCLient 12/23/23
int mqttKeepAlive = 30; // publish temp every x seconds to keep MQTT client connected
// Publishing Topics 
char topic[60];
char topicBase[60];
#define topic_base_path "msb/traffic/GateCounter"
#define MQTT_PUB_HELLO "msb/traffic/GateCounter/hello"
#define MQTT_PUB_TEMP "msb/traffic/GateCounter/temp"
#define MQTT_PUB_TIME "msb/traffic/GateCounter/time"
#define MQTT_PUB_EXIT_CARS "msb/traffic/GateCounter/ExitTotal"
#define MQTT_PUB_CARS_HOURLY  "msb/traffic/GateCounter/Cars"
#define MQTT_PUB_INPARK_CARS "msb/traffic/GateCounter/inParkCars"
#define MQTT_PUB_SUMMARY "msb/traffic/GateCounter/Summary"
#define MQTT_PUB_DAYOFMONTH "msb/traffic/GateCounter/DayOfMonth"
#define MQTT_PUB_SHOWTOTAL "msb/traffic/GateCounter/ShowTotal"
#define MQTT_PUB_TTP "msb/traffic/GateCounter/TTP"
#define MQTT_PUB_BEAM_SENSOR_STATE "msb/traffic/GateCounter/beamSensorState"
#define MQTT_PUB_MAG_SENSOR_STATE "msb/traffic/GateCounter/magSensorState"
#define MQTT_PUB_DAYSRUNNING "msb/traffic/GateCounter/DaysRunning"
#define MQTT_DEBUG_LOG "msb/traffic/GateCounter/debuglog"
#define MQTT_COUNTER_LOG "msb/traffic/GateCounter/CounterLog"
#define MQTT_PUB_MAGBEAM_MS "msb/traffic/GateCounter/mag-beam_ms"
#define MQTT_PUB_BETWEENCARS_MS "msb/traffic/GateCounter/betweenCars"
#define MQTT_PUB_BEAMHIGH_MS "msb/traffic/GateCounter/beam-high_ms"
// Subscribing Topics (to reset values)
#define MQTT_SUB_TOPIC0  "msb/traffic/CarCounter/EnterTotal"          // get enter counts from carCounter
#define MQTT_SUB_TOPIC1  "msb/traffic/GateCounter/resetDailyCount"    // Reset Daily counter
#define MQTT_SUB_TOPIC2  "msb/traffic/GateCounter/resetShowCount"     // Resets Show Counter
#define MQTT_SUB_TOPIC3  "msb/traffic/GateCounter/resetDayOfMonth"    // Reset Calendar Day
#define MQTT_SUB_TOPIC4  "msb/traffic/GateCounter/resetDaysRunning"   // Reset Days Running
#define MQTT_SUB_TOPIC5  "msb/traffic/GateCounter/gateCounterTimeout" // Reset Timeout if car leaves detection Zone
#define MQTT_SUB_CARMS  "msb/traffic/GateCounter/carDetectMS"       // Reset sync time from magSensor trip to beamSensor Active
#define MQTT_SUB_LOGGING "msb/traffic/GateCounter/loggingEnabled"     // toggle logging function

// State Machine for Car Counting
enum CarDetectionState {
    IDLE,
    MAG_SENSOR_TRIGGERED,
    BEAM_SENSOR_VALIDATION,
    VEHICLE_CONFIRMED
};
CarDetectionState carDetectionState = IDLE;

unsigned long magSensorTripTime = 0;     // Time when the magnetometer was first triggered
unsigned long beamSensorTripTime = 0;    // Time when the beam sensor was first triggered
unsigned long lastCarExitTime = 0;       // Time when the last car exited (beam sensor went low)
static int carDetectMS = 1200;           // Minimum wait duration for a vehicle to be confirmed
unsigned long timeToPassMS = 0;          // Time from start of detection to confirmation
bool magSensorWasTriggered = false;      // Tracks if the magnetic sensor was triggered recently
bool carPassed = false;                  // Tracks if a car has fully passed through

// Track previous states for efficient MQTT publishing
int prevMagSensorState = -1;  // Start with -1 to ensure initial publishing
int prevBeamSensorState = -1; // Start with -1 to ensure initial publishing

AsyncWebServer server(80);     // Define Webserver
String currentDirectory = "/"; // Current working directory

unsigned long ota_progress_millis = 0;

//void saveHourlyCounts();  // forward declaration

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  //saveHourlyCounts();
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
    // Log when OTA has finished
    if (success) {
      Serial.println("OTA update finished successfully!");
    } else {
      Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
}

/** REAL TIME Clock & Time Related Variables **/
RTC_DS3231 rtc;
const char* ampm ="AM";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;
//float tempF = 0.0;

// Initialize DHT sensor & Variables for temperature and humidity readTempandRH()
DHT dht(DHTPIN, DHTTYPE);
float tempF = 0.0;  // Temperature
float humidity = 0.0;     // Humidity

/** Display Definitions & variables **/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
/*Line Numbers used for Display*/
const int line1 =0;
const int line2 =9;
const int line3 = 19;
const int line4 = 30;
const int line5 = 42;
const int line6 = 50;
const int line7 = 53;

//Create Multiple WIFI Object
WiFiMulti wifiMulti;
//WiFiClientSecure espGateCounter;
WiFiClient espGateCounter;

//const uint32_t connectTimeoutMs = 10000;
uint16_t connectTimeOutPerAP=5000;

/***** MQTT Setup Variables  *****/
PubSubClient mqtt_client(espGateCounter);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (500)
char msg[MSG_BUFFER_SIZE];
//char mqtt_server[] = mqtt_Server;
//char mqtt_username[] = mqtt_UserName;
//char mqtt_password[] = mqtt_Password;
//const int mqtt_port = mqtt_Port;
bool loggingEnabled = false; // Default: Logging is OFF
bool mqtt_connected = false;
bool wifi_connected = false;
int wifi_connect_attempts = 5;
bool hasRun = false;

/***** GLOBAL VARIABLES *****/
unsigned int dayOfMonth;      // Current Calendar day
unsigned int lastDayOfMonth;  // Last calendar day used to reset days running
unsigned int currentHr12;     // Current Hour 12 Hour format
unsigned int currentHr24;     // Current Hour 24 Hour Format
unsigned int currentMin;      // Current Minute
unsigned int currentSec;      // Current Second
unsigned int daysRunning;     // Number of days the show is running.
unsigned int currentTimeMinute; // for converting clock time hh:mm to clock time min since midnight
int totalDailyCars; // total cars counted per day 24/7 Needed for debugging
int totalShowCars;  // total cars counted for durning show hours open (5:00 pm to 9:10 pm)
int inParkCars;     // cars in park Enter Cars - Exit Cars
int carCounterCars; // Counts from Car Counter
int lastcarCounterCars; // Used to publish in park cars when car counter increases
int magSensorState, lastmagSensorState ; /* Store states of Mag Sensor*/
int beamSensorState, lastbeamSensorState ; /* Store states of Beam Sensor */
unsigned long triggerTime; // Stores the time when sensor 1 is triggered
unsigned long beamSensorAlarm; // Monitor time Beam Sensor is blocked


/***** TIME VARIABLES *****/
const unsigned long wifi_connectioncheckMillis = 5000; // check for connection every 5 sec
const unsigned long mqtt_connectionCheckMillis = 30000; // check for connection
unsigned long start_MqttMillis; // for Keep Alive Timer
unsigned long start_WiFiMillis; // for keep Alive Timer
int gateCounterTimeout = 60000; // default time for car counter alarm in millis
char buf2[25] = "YYYY-MM-DD hh:mm:ss"; // time car detected

//***** DAILY RESET FLAGS *****
bool flagDaysRunningReset = false;
bool flagMidnightReset = false;
bool flagDailyShowStartReset = false;
bool flagDailySummarySaved = false;
bool flagDailyShowSummarySaved = false;
bool flagHourlyReset = false;
bool showTime = false;
bool resetFlagsOnce = false;

// **********FILE NAMES FOR SD CARD *********
File myFile; //used to write files to SD Card
const String fileName1 = "/ExitTotal.txt"; // /DailyTot.txt file to store daily counts in the event of a Failure
const String fileName2 = "/ShowTotal.txt";  // /ShowTot.txt file to store season total counts
const String fileName3 = "/DayOfMonth.txt"; // /DayOfMonth.txt file to store current day number
const String fileName4 = "/RunDays.txt"; // /RunDays.txt file to store days since open
const String fileName5 = "/GateHourlyData.csv"; // /GateSummary.csv Stores Daily Totals by Hour and total
const String fileName6 = "/GateLog.csv"; // GateLog.csv file to store all car counts for season (was MASTER.CSV)
const String fileName7 = "/GateDailySummary.csv"; // Show summary of counts during show (5:00pm to 9:10pm)
const String fileName8 = "/data/index.html"; // data folder and index.html for serving files OTA
const String fileName9 = "/data/style.css"; // data folder and index.html for serving files OTA
const String fileName10 = "/SensorLog.csv"; // sensorLog.csv for recording gate sensors for plotting
//const String fileName7 = "/SensorBounces.csv"; // /SensorBounces.csv file to store all bounce counts for season

/***** Arrays for Hourly Totals/Averages *****/
static unsigned int hourlyCount[24] = {0}; // Array for Daily total cars per hour
static float hourlyTemp[24] = {0.0};   // Array to store average temperatures for 24 hours

/***** Arrays Used to make display Pretty *****/
static char days[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
static char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

// sync Time at REBOOT
void SetLocalTime()  {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time. Using Compiled Date");
    return;
  }
  //Following used for Debugging and can be commented out
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of week: ");
  Serial.println(&timeinfo, "%A");
  Serial.print("Month: ");
  Serial.println(&timeinfo, "%B");
  Serial.print("Day of Month: ");
  Serial.println(&timeinfo, "%d");
  Serial.print("Year: ");
  Serial.println(&timeinfo, "%Y");
  Serial.print("Hour: ");
  Serial.println(&timeinfo, "%H");
  Serial.print("Hour (12 hour format): ");
  Serial.println(&timeinfo, "%I");
  Serial.print("Minute: ");
  Serial.println(&timeinfo, "%M");
  Serial.print("Second: ");
  Serial.println(&timeinfo, "%S");
  Serial.println("Time variables");
  char timeHour[3];
  strftime(timeHour,3, "%H", &timeinfo);
  Serial.println(timeHour);
  char timeWeekDay[10];
  strftime(timeWeekDay,10, "%A", &timeinfo);
  Serial.println(timeWeekDay);
  Serial.println();

  // Convert NTP time string to set RTC
  char timeStringBuff[50]; //50 chars should be enough
  strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.println(timeStringBuff);
  rtc.adjust(DateTime(timeStringBuff));
}

void setup_wifi()  {
    Serial.println("Connecting to WiFi");
    display.println("Connecting to WiFi..");
    display.display();
    while(wifiMulti.run(connectTimeOutPerAP) != WL_CONNECTED) {
        Serial.print(".");
    }
    Serial.println("Connected to the WiFi network");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.display();
  
    display.setCursor(0, line1);
    display.print("SSID: ");
    display.println(WiFi.SSID());   // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    
    IPAddress ip = WiFi.localIP();  // print your board's IP address:
    Serial.print("IP: ");
    Serial.println(ip);
    display.setCursor(0, line2);
    display.print("IP: ");
    display.println(ip);
    
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
    display.setCursor(0, line3);
    display.print("signal: ");
    display.print(rssi);  // print the received signal strength:
    display.println(" dBm");
    display.display();
 
    delay(1000);
}  // END WiFi Setup

// BEGIN OTA SD Card File Operations
void listSDFiles(AsyncWebServerRequest *request) {
    String fileList = "Files in " + currentDirectory + ":\n";

    File root = SD.open(currentDirectory);
    if (!root || !root.isDirectory()) {
        request->send(500, "text/plain", "Failed to open directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        fileList += String(file.name()) + " (" + String(file.size()) + " bytes)\n";
        file = root.openNextFile();
    }

    request->send(200, "text/plain", fileList);
}

void downloadSDFile(AsyncWebServerRequest *request) {
    if (!request->hasParam("filename")) {
        request->send(400, "text/plain", "Filename is required");
        return;
    }

    String filename = currentDirectory + request->getParam("filename")->value();
    if (!SD.exists(filename)) {
        request->send(404, "text/plain", "File not found");
        return;
    }

    // Add Content-Disposition header for proper filename handling
    AsyncWebServerResponse *response = request->beginResponse(SD, filename, "application/octet-stream");
    response->addHeader("Content-Disposition", "attachment; filename=\"" + String(request->getParam("filename")->value()) + "\"");

    request->send(response);
}

void uploadSDFile(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    static File uploadFile; // Keep track of the currently uploading file
    String fullPath = currentDirectory + "/" + filename; // Respect the current directory

    // Handle the start of the upload
    if (index == 0) {
        Serial.printf("Upload started: %s\n", fullPath.c_str());
        if (SD.exists(fullPath)) {
            SD.remove(fullPath); // Remove the file if it already exists
        }
        uploadFile = SD.open(fullPath, FILE_WRITE);
        if (!uploadFile) {
            Serial.printf("Failed to open file: %s\n", fullPath.c_str());
            request->send(500, "text/plain", "Failed to open file for writing");
            return;
        }
    }

    // Write data to the file
    if (uploadFile) {
        uploadFile.write(data, len);
    }

    // Handle the end of the upload
    if (final) {
        if (uploadFile) {
            uploadFile.close();
            Serial.printf("Upload completed: %s\n", fullPath.c_str());
            request->send(200, "text/plain", "File uploaded successfully to " + currentDirectory);
        } else {
            Serial.printf("Upload failed: %s\n", fullPath.c_str());
            request->send(500, "text/plain", "Failed to write file");
        }
    }
}

void changeDirectory(AsyncWebServerRequest *request) {
    if (!request->hasParam("dir")) {
        request->send(400, "text/plain", "Directory name is required");
        return;
    }

    String newDirectory = request->getParam("dir")->value();
    if (newDirectory[0] != '/') {
        newDirectory = currentDirectory + "/" + newDirectory;
    }

    if (SD.exists(newDirectory) && SD.open(newDirectory).isDirectory()) {
        currentDirectory = newDirectory;
        request->send(200, "text/plain", "Changed directory to " + currentDirectory);
    } else {
        request->send(404, "text/plain", "Directory not found");
    }
}

void deleteSDFile(AsyncWebServerRequest *request) {
    if (!request->hasParam("filename")) {
        request->send(400, "text/plain", "Filename is required");
        return;
    }

    String fileName = request->getParam("filename")->value();
    String fullPath = currentDirectory + "/" + fileName; // Respect the current directory

    // Normalize the file path
    if (fullPath.startsWith("//")) {
        fullPath = fullPath.substring(1); // Remove redundant leading slashes
    }

    if (SD.exists(fullPath)) {
        if (SD.remove(fullPath)) {
            Serial.printf("File deleted: %s\n", fullPath.c_str());
            request->send(200, "text/plain", "File deleted successfully: " + fullPath);
        } else {
            Serial.printf("Failed to delete file: %s\n", fullPath.c_str());
            request->send(500, "text/plain", "Failed to delete file: " + fullPath);
        }
    } else {
        Serial.printf("File not found: %s\n", fullPath.c_str());
        request->send(404, "text/plain", "File not found: " + fullPath);
    }
}
//END OTA SD Card File Operations

// HTML Content now served from /data/index.html and /data/style.css
void setupServer() {
    // Serve HTML file
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!SD.exists("/data/index.html")) {
            request->send(500, "text/plain", "index.html not found in /data");
            return;
        }
        request->send(SD, "/data/index.html", "text/html");
    });

    // Serve CSS file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!SD.exists("/data/style.css")) {
            request->send(500, "text/plain", "style.css not found in /data");
            return;
        }
        request->send(SD, "/data/style.css", "text/css");
    });

    // Setup Web Server Routes
    server.on("/listFiles", HTTP_GET, listSDFiles);
    server.on("/download", HTTP_GET, downloadSDFile);
    server.on("/upload", HTTP_POST, 
        [](AsyncWebServerRequest *request) {},
        uploadSDFile);
    // Handle file uploads to /data directory
    server.on("/uploadToData", HTTP_POST, 
        [](AsyncWebServerRequest *request) {},
        [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            String fullPath = "/data/" + filename;
            static File uploadFile;

            if (index == 0) { // First chunk
                if (SD.exists(fullPath)) {
                    SD.remove(fullPath);
                }
                uploadFile = SD.open(fullPath, FILE_WRITE);
                if (!uploadFile) {
                    request->send(500, "text/plain", "Failed to open file for writing");
                    return;
                }
            }

            if (uploadFile) { // Write the data
                uploadFile.write(data, len);
            }

            if (final) { // Final chunk
                if (uploadFile) {
                    uploadFile.close();
                }
                request->send(200, "text/plain", "File uploaded successfully to /data");
            }
        });
    
    server.on("/changeDirectory", HTTP_GET, changeDirectory);
    server.on("/ShowSummary.csv", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response = request->beginResponse(SD, "/ShowSummary.csv", "text/csv");
        response->addHeader("Access-Control-Allow-Origin", "*");
        request->send(response);
    });

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hi! This is The Car Counter.");
    });

    // Elegant OTA
    // You can also enable authentication by uncommenting the below line.
    // ElegantOTA.setAuth("admin", "password");
    ElegantOTA.setID(THIS_MQTT_CLIENT);  // Set Hardware ID
    ElegantOTA.setFWVersion(FWVersion);   // Set Firmware Version
    ElegantOTA.setTitle(OTA_Title);  // Set OTA Webpage Title
    //ElegantOTA.setFilesystemMode(true);  // added 10.16.24.4
    // Start ElegantOTA
    ElegantOTA.begin(&server);    // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);
    server.begin();
    Serial.println("HTTP server started");
}

/***** MQTT SECTION ******/
// Save messages if MQTT is not connected in Queue
std::queue<String> publishQueue;

// Used to publish MQTT Messages
void publishMQTT(const char *topic, const String &message) {
    if (mqtt_client.connected()) {
        mqtt_client.publish(topic, message.c_str());
    } else {
        Serial.printf("MQTT not connected. Adding to queue: %s -> %s\n", topic, message.c_str());
        publishQueue.push(String(topic) + "|" + message);  // Add message to queue
    }
    start_MqttMillis = millis();
}

// Used to publish Queued Messages
void publishQueuedMessages() {
    while (!publishQueue.empty() && mqtt_client.connected()) {
        String data = publishQueue.front();
        publishQueue.pop();
        
        int delimiterPos = data.indexOf('|');
        if (delimiterPos != -1) {
            String topic = data.substring(0, delimiterPos);
            String message = data.substring(delimiterPos + 1);
            mqtt_client.publish(topic.c_str(), message.c_str());
        }
    }
}

void publishDebugLog(const String &message) {
    publishMQTT(MQTT_DEBUG_LOG, message);
}

// Used to publish current counts to update Car Counter every 30 seconds if no car is counted
void KeepMqttAlive() {
   publishMQTT(MQTT_PUB_TEMP, String(tempF));
   publishMQTT(MQTT_PUB_EXIT_CARS, String(totalDailyCars));
   publishMQTT(MQTT_PUB_INPARK_CARS, String(inParkCars));
   start_MqttMillis = millis();
}

// Forward Declare the callback function
void callback(char* topic, byte* payload, unsigned int length);

//Connects to MQTT Server
void MQTTreconnect() {
    static unsigned long lastReconnectAttempt = 0; // Tracks the last reconnect attempt time
    const unsigned long reconnectInterval = 5000; // Time between reconnect attempts (5 seconds)

    // If the client is already connected, do nothing
    if (mqtt_client.connected()) {
        return;
    }

    // Check if enough time has passed since the last attempt
    if (millis() - lastReconnectAttempt > reconnectInterval) {
        lastReconnectAttempt = millis(); // Update the last attempt time
        Serial.println("Attempting MQTT connection...");

        for (int i = 0; i < mqtt_servers_count; i++) {
            // Set the server for the current configuration
            mqtt_client.setServer(mqtt_configs[i].server, mqtt_configs[i].port);
            mqtt_client.setCallback(callback);  // required to receive messages

            // Create a unique client ID
            String clientId = THIS_MQTT_CLIENT;

            // Attempt to connect using the current server's credentials
            Serial.printf("Trying MQTT server: %s:%d\n", mqtt_configs[i].server, mqtt_configs[i].port);
            if (mqtt_client.connect(clientId.c_str(), mqtt_configs[i].username, mqtt_configs[i].password)) {
                Serial.printf("Connected to MQTT server: %s\n", mqtt_configs[i].server);
                
                // Display connection status
                display.setTextSize(1);
                display.setTextColor(WHITE);
                display.setCursor(0,line5);
                display.println("MQTT Connect");
                display.display();
                Serial.println("connected!");
                Serial.println("Waiting for Car");                
                // Once connected, publish an announcement
                //publishMQTT(MQTT_PUB_HELLO, "Gate Counter ONLINE on " + String(mqtt_configs[i].server));
                publishMQTT(MQTT_PUB_HELLO, "Gate Counter ONLINE!");
                publishMQTT(MQTT_PUB_TEMP, String(tempF));
                publishMQTT(MQTT_PUB_EXIT_CARS, String(totalDailyCars));
                publishMQTT(MQTT_PUB_SHOWTOTAL, String(totalShowCars));

                // Subscribe to necessary topics
                mqtt_client.subscribe(MQTT_PUB_HELLO);
                mqtt_client.subscribe(MQTT_SUB_TOPIC0);
                mqtt_client.subscribe(MQTT_SUB_TOPIC1);
                mqtt_client.subscribe(MQTT_SUB_TOPIC2);
                mqtt_client.subscribe(MQTT_SUB_TOPIC3);
                mqtt_client.subscribe(MQTT_SUB_TOPIC4);
                mqtt_client.subscribe(MQTT_SUB_TOPIC5);
                mqtt_client.subscribe(MQTT_SUB_CARMS);
                mqtt_client.subscribe(MQTT_SUB_LOGGING);

                // Log subscriptions
                Serial.println("Subscribed to MQTT topics.");
                publishMQTT(MQTT_DEBUG_LOG, "MQTT connected and topics subscribed.");

                return; // Exit loop on successful connection
            } else {
                // Log connection failure for the current server
                Serial.printf("Failed to connect to MQTT server: %s (rc=%d)\n", mqtt_configs[i].server, mqtt_client.state());
            }
        }

        // If all servers fail
        Serial.println("All MQTT server attempts failed. Will retry...");
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, line6);
        display.println("MQTT Error");
        display.display();
    }
}
/***** END MQTT SECTION *****/

void checkWiFiConnection() {

/* non-blocking WiFi and MQTT Connectivity Checks 
    First check if WiFi is connected */
    if (wifiMulti.run() == WL_CONNECTED) {
        /* If MQTT is not connected then Attempt MQTT Connection */
        if (!mqtt_client.connected()) {
            Serial.print("hour = ");
            Serial.println(currentHr12);
            Serial.println("Attempting MQTT Connection");
            MQTTreconnect();
            start_MqttMillis = millis();
        } else {
                //keep MQTT client connected when WiFi is connected
                mqtt_client.loop();
        }
    } else {
        // If WiFi if lost, then attemp non blocking WiFi Connection
        if ((millis() - start_WiFiMillis) > wifi_connectioncheckMillis) {
            setup_wifi();
            start_WiFiMillis = millis();
        }
    }    
}

// =========== GET SAVED SETUP FROM SD CARD ==========
// open DAILYTOT.txt to get initial dailyTotal value
void getDailyTotal()   {
    // open DAILYTOT.txt to get initial dailyTotal value
    myFile = SD.open(fileName1,FILE_READ);
    if (myFile) {
        while (myFile.available()) {
            totalDailyCars = myFile.parseInt(); // read total
            Serial.print(" Daily cars from file = ");
            Serial.println(totalDailyCars);
        }
        myFile.close();
        publishMQTT(MQTT_PUB_EXIT_CARS, String(totalDailyCars));
    }  
    else  {
        Serial.print("SD Card: Cannot open the file: ");
        Serial.println(fileName1);
    }
} // end getDailyTotal

/** Get season total cars since show opened */
void getShowTotal() {
  myFile = SD.open(fileName2,FILE_READ);
  if (myFile) {
    while (myFile.available()) {
      totalShowCars = myFile.parseInt(); // read total
      Serial.print(" Total Show cars from file = ");
      Serial.println(totalShowCars);
    }
    myFile.close();
    publishMQTT(MQTT_PUB_SHOWTOTAL, String(totalShowCars));
  } else {
    Serial.print(F("SD Card: Cannot open the file: "));
    Serial.println(fileName2);
  }
}

// get the last calendar day used for reset daily counts)
void getDayOfMonth() {
    myFile = SD.open(fileName3,FILE_READ);
    if (myFile) {
      while (myFile.available()) {
        lastDayOfMonth = myFile.parseInt(); // read day Number
        Serial.print(" Calendar Day = ");
        Serial.println(lastDayOfMonth);
      }
      myFile.close();
      publishMQTT(MQTT_PUB_DAYOFMONTH, String(lastDayOfMonth));
    } else {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName3);
    }
} 

// Days the show has been running)
void getDaysRunning() {
  myFile = SD.open(fileName4,FILE_READ);
  if (myFile) {
    while (myFile.available()) {
      daysRunning = myFile.parseInt(); // read day Number
      Serial.print(" Days Running = ");
      Serial.println(daysRunning);
    }
    myFile.close();
    publishMQTT(MQTT_PUB_DAYSRUNNING, String(daysRunning));
  } else {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName4);
  }
} 

/** Get hourly car counts on reboot */
void getHourlyData() {
    DateTime now = rtc.now();
    char dateBuffer[13];
    snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", now.year(), now.month(), now.day());

    // Open the file for reading
    File file = SD.open(fileName5, FILE_READ);
    if (!file) {
        Serial.println("Failed to open GateHourlyData.csv. Resetting hourly data.");
        publishMQTT(MQTT_DEBUG_LOG, "Failed to open GateHourlyData.csv. Resetting hourly data.");
        memset(hourlyCount, 0, sizeof(hourlyCount)); // Reset to zeros
        return;
    }

    bool rowFound = false;

    // Read the file line by line
    while (file.available()) {
        String line = file.readStringUntil('\n');
        if (line.startsWith(dateBuffer)) {
            // Parse the row for the current date
            rowFound = true;

            int parsedValues = sscanf(line.c_str(),
                                      "%*[^,],%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                      &hourlyCount[0], &hourlyCount[1], &hourlyCount[2], &hourlyCount[3],
                                      &hourlyCount[4], &hourlyCount[5], &hourlyCount[6], &hourlyCount[7],
                                      &hourlyCount[8], &hourlyCount[9], &hourlyCount[10], &hourlyCount[11],
                                      &hourlyCount[12], &hourlyCount[13], &hourlyCount[14], &hourlyCount[15],
                                      &hourlyCount[16], &hourlyCount[17], &hourlyCount[18], &hourlyCount[19],
                                      &hourlyCount[20], &hourlyCount[21], &hourlyCount[22], &hourlyCount[23]);

            if (parsedValues == 24) {
                Serial.println("Successfully loaded hourly data for today.");
                publishMQTT(MQTT_DEBUG_LOG, "Successfully loaded hourly data for today.");
                for (int i = 0; i < 24; i++) {
                    Serial.printf("Hour %02d: %d cars\n", i, hourlyCount[i]);
                    char debugMsg[50];
                    snprintf(debugMsg, sizeof(debugMsg), "Hour %02d: %d cars", i, hourlyCount[i]);
                    publishMQTT(MQTT_DEBUG_LOG, String(debugMsg));
                }
            } else {
                Serial.println("Error parsing today's row. Resetting hourly data.");
                publishMQTT(MQTT_DEBUG_LOG, "Error parsing today's row. Resetting hourly data.");
                memset(hourlyCount, 0, sizeof(hourlyCount)); // Reset to zeros
            }
            break; // Exit loop after processing today's row
        }
    }
    file.close();

    if (!rowFound) {
        Serial.println("No data for today found. Resetting hourly data.");
        publishMQTT(MQTT_DEBUG_LOG, "No data for today found. Resetting hourly data.");
        memset(hourlyCount, 0, sizeof(hourlyCount)); // Reset to zeros
    }
}

/***** UPDATE and SAVE TOTALS TO SD CARD *****/
/** Save the daily Total of cars counted */
void saveDailyTotal() {
  myFile = SD.open(fileName1,FILE_WRITE);
  if (myFile) {  // check for an open failure
     myFile.print(totalDailyCars);
     myFile.close();
     publishMQTT(MQTT_PUB_EXIT_CARS, String(totalDailyCars));
     publishMQTT(MQTT_PUB_INPARK_CARS, String(inParkCars));
  } else {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName1);
  } 
  publishMQTT(MQTT_PUB_EXIT_CARS, String(totalDailyCars));
}

/* Save the grand total cars file for season  */
void saveShowTotal() {  
    myFile = SD.open(fileName2,FILE_WRITE);
    if (myFile) {
        myFile.print(totalShowCars);
        myFile.close();
    } else {
        Serial.print(F("SD Card: Cannot open the file: "));
        Serial.println(fileName2);
    }
    publishMQTT(MQTT_PUB_SHOWTOTAL, String(totalShowCars));  
}

// Save the calendar day to file ----- */
void saveDayOfMonth() {
    myFile = SD.open(fileName3,FILE_WRITE);
    if (myFile) {
        myFile.print(dayOfMonth);
        myFile.close();
      } else {
        Serial.print(F("SD Card: Cannot open the file: "));
        Serial.println(fileName3);
    }
      publishMQTT(MQTT_PUB_DAYOFMONTH, String(dayOfMonth));
}

/** Save number of days the show has been running */
void saveDaysRunning() {
    myFile = SD.open(fileName4,FILE_WRITE);
    if (myFile) {
      myFile.print(daysRunning);
      myFile.close();
    } else {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName4);
    }
    publishMQTT(MQTT_PUB_DAYSRUNNING, String(daysRunning));
}

// Save cars counted each hour in the event of a reboot
// Refactored saveHourlyCounts function
void saveHourlyCounts() {
    DateTime now = rtc.now();
    char dateBuffer[13];
    snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", now.year(), now.month(), now.day());

    int currentHour = now.hour(); // Get the current hour (0-23)

    File file = SD.open(fileName5, FILE_READ);
    String updatedContent = "";
    bool rowExists = false;

    if (file) {
        while (file.available()) {
            String line = file.readStringUntil('\n');
            if (line.startsWith(dateBuffer)) {
                rowExists = true;
                updatedContent += dateBuffer; // Start with the date
                int lastCommaIndex = line.indexOf(",") + 1; // Start after the date
 
                // Parse each value in the line
                for (int i = 0; i < 24; i++) {
                    int nextCommaIndex = line.indexOf(",", lastCommaIndex);
                    String currentValue = (nextCommaIndex != -1) 
                                           ? line.substring(lastCommaIndex, nextCommaIndex)
                                           : line.substring(lastCommaIndex);
                    lastCommaIndex = (nextCommaIndex != -1) ? nextCommaIndex + 1 : lastCommaIndex;

                    // Replace value for the current hour
                    updatedContent += (i == currentHour) 
                                      ? "," + String(hourlyCount[i]) 
                                      : "," + currentValue;
                }
                updatedContent += "\n";

                // Publish current hour's data to MQTT
                char topic[60];
                snprintf(topic, sizeof(topic), "%s/hour%02d", MQTT_PUB_CARS_HOURLY, currentHour);
                publishMQTT(topic, String(hourlyCount[currentHour]));
            } else {
                updatedContent += line + "\n"; // Preserve other rows
            }
        }
        file.close();
    }

    // If no row exists for today, create a new one
    if (!rowExists) {
        updatedContent += dateBuffer;
        for (int i = 0; i < 24; i++) {
            updatedContent += (i == currentHour) ? "," + String(hourlyCount[i]) : ",0";
        }
        updatedContent += "\n";

        // Publish current hour's data to MQTT
        char topic[60];
        snprintf(topic, sizeof(topic), "%s/hour%02d", MQTT_PUB_CARS_HOURLY, currentHour);
        publishMQTT(topic, String(hourlyCount[currentHour]));

        // Publish debug log
        char debugMessage[100];
        snprintf(debugMessage, sizeof(debugMessage), "Hourly data saved for hour %02d.", currentHour);
        publishMQTT(MQTT_DEBUG_LOG, debugMessage);
    }

    // Write updated content back to the file
    file = SD.open(fileName5, FILE_WRITE);
    if (file) {
        file.print(updatedContent);
        file.close();
        Serial.printf("Hourly counts for hour %02d saved and published.\n", currentHour);
    } else {
        Serial.println("Failed to open GateHourlyData.csv for writing.");
    }
}

// Save and Publish Show Totals
void saveDailyShowSummary() {
    DateTime now = rtc.now();

    // Define show hours (5 PM to 9 PM)
    //const int showStartHour = 17; // 5 PM
    //const int showEndHour = 20;  // Up to 9 PM

    // Calculate cumulative totals for each key hour
    int cumulative6PM = hourlyCount[17]; // Total at 6 PM
    int cumulative7PM = cumulative6PM + hourlyCount[18]; // Total at 7 PM
    int cumulative8PM = cumulative7PM + hourlyCount[19]; // Total at 8 PM
    int cumulative9PM = cumulative8PM + hourlyCount[20]; // Total at 9 PM

    // Calculate total cars counted before the show starts
    int totalBefore5 = 0;
    for (int i = 0; i < 17; i++) { // Loop from hour 0 to hour 16
        totalBefore5 += hourlyCount[i];
    }

    // Include additional cars detected between 9:00 PM and 9:20 PM
    if (now.hour() * 60 + now.minute() <= showEndMin) {
        cumulative9PM += hourlyCount[21];
    }

    // Calculate the average temperature during show hours (5 PM to 9 PM)
    float showTempSum = 0.0;
    int showTempCount = 0;

    for (int i = 17; i <= 20; i++) { // Loop only between 5 PM and 9 PM
        if (hourlyTemp[i] != 0.0) { // Include valid temperature readings
            showTempSum += hourlyTemp[i];
            showTempCount++;
        }
    }

    float showAverageTemp = (showTempCount > 0) ? (showTempSum / showTempCount) : 0.0;

    // Open file for appending
    File summaryFile = SD.open(fileName7, FILE_APPEND);
    if (!summaryFile) {
        Serial.println("Failed to open daily show summary file.");
        publishMQTT(MQTT_DEBUG_LOG, "Failed to open daily show summary file.");
        return;
    }

    // Format the current date
    char dateBuffer[13];
    snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", now.year(), now.month(), now.day());

    // Append the show summary data
    summaryFile.printf("%s,%d,%d,%d,%d,%d,%d,%d,%.1f\n",
                       dateBuffer,       // Current date
                       daysRunning,      // Total days running
                       totalBefore5,     // Cars counted before 5pm
                       cumulative6PM,    // Cumulative total up to 6 PM
                       cumulative7PM,    // Cumulative total up to 7 PM
                       cumulative8PM,    // Cumulative total up to 8 PM
                       cumulative9PM,    // Cumulative total up to 9 PM, including 9:10 PM cars
                       totalShowCars,    // Total show cars
                       showAverageTemp); // Average temperature during show hours
    summaryFile.close();

    // Publish show summary data to MQTT
    publishMQTT(MQTT_PUB_SUMMARY, String("Date: ") + dateBuffer +
                                        ", DaysRunning: " + daysRunning +
                                        ", Before5: " + totalBefore5 +
                                        ", 6PM: " + cumulative6PM +
                                        ", 7PM: " + cumulative7PM +
                                        ", 8PM: " + cumulative8PM +
                                        ", 9PM: " + cumulative9PM +
                                        ", ShowTotal: " + totalShowCars +
                                        ", ShowAvgTemp: " + String(showAverageTemp, 1));

    Serial.printf("Daily show summary written: %s, DaysRunning: %d, Before5: %d, 6PM: %d, 7PM: %d, 8PM: %d, 9PM: %d, ShowTotal: %d, Avg Temp: %.1f°F.\n",
                  dateBuffer, daysRunning, totalBefore5, cumulative6PM, cumulative7PM, cumulative8PM, cumulative9PM, totalShowCars, showAverageTemp);
}

void getSavedValuesOnReboot() {
    DateTime now = rtc.now();

    // Read the last recorded day from the SD card
    getDayOfMonth();

    // Check if the ESP32 is rebooting on a new day
    if (now.day() != lastDayOfMonth) {
        dayOfMonth = now.day(); // Update to the current day
        saveDayOfMonth(); // Save the new day to the SD card
        totalDailyCars = 0; // Reset daily car count
        saveDailyTotal(); // Save the reset value to the SD card

        // Increment days running, except on Christmas Eve
        if (!(now.month() == 12 && now.day() == 24) ) {
            daysRunning++;
            saveDaysRunning(); // Save updated days running to the SD card
            publishMQTT(MQTT_DEBUG_LOG, "Rebooted, Day of Month Changed, Days Running Increased.");
        }

        // Log the update
        Serial.println("ESP32 reboot detected on a new day. Counts reset/updated.");
        publishMQTT(MQTT_DEBUG_LOG, "Rebooted, Counts reset/updated for new day.");
    } else {
        // If the day has not changed, reload the existing totals
        getDailyTotal();   // Reload daily car count
        getShowTotal();    // Reload show total
        getDaysRunning();  // Reload days running
        getHourlyData();   // Reload Hourly Count Data

        // Log the reload
        Serial.println("ESP32 reboot detected on the same day. Reloading saved counts.");
        publishMQTT(MQTT_DEBUG_LOG, "Rebooted, Counts reloaded for same day.");
    }
}
/***** END OF DATA STORAGE & RETRIEVAL OPS *****/

/*** MQTT CALLBACK TOPICS ****/
void callback(char* topic, byte* payload, unsigned int length) {

  char message[length + 1];
  strncpy(message, (char*)payload, length);
  message[length] = '\0'; // Safely null-terminate the payload
  
  if (strcmp(topic, MQTT_SUB_TOPIC0) == 0)  {
    /* Receive MQTT message with updated CarCounter totals*/
    //carCounterCars = atoi((char *)payload);
    carCounterCars = atoi(message);
    inParkCars=carCounterCars-totalDailyCars; // recalculate cars in park
    if (carCounterCars != lastcarCounterCars) {
      publishMQTT(MQTT_PUB_EXIT_CARS, String(totalDailyCars));
      publishMQTT(MQTT_PUB_INPARK_CARS, String(inParkCars)); // update in park cars
      lastcarCounterCars = carCounterCars;
    }  
  } else if (strcmp(topic, MQTT_SUB_TOPIC1) == 0) {
    /* Topic used to manually reset gate total cars */
    //totalDailyCars = atoi((char *)payload);
    totalDailyCars = atoi(message);
    saveDailyTotal();
    Serial.println(F(" Gate Counter Updated"));
    publishMQTT(MQTT_PUB_HELLO, "Daily Total Updated");
  } else if (strcmp(topic, MQTT_SUB_TOPIC2) == 0) {
    /* Topic used to manually reset Total Show Cars */
    //totalShowCars = atoi((char *)payload);
    totalShowCars = atoi (message);
    saveShowTotal();
    Serial.println(F(" Show Counter Updated"));
    publishMQTT(MQTT_PUB_HELLO, "Show Counter Updated");
  } else if (strcmp(topic, MQTT_SUB_TOPIC3) == 0) {
    /* Topic used to manually reset Calendar Day */
    //DayOfMonth = atoi((char *)payload);
    dayOfMonth = atoi(message);
    saveDayOfMonth();
    Serial.println(F(" Calendar Day of Month Updated"));
    publishMQTT(MQTT_PUB_HELLO, "Calendar Day Updated");
  } else if (strcmp(topic, MQTT_SUB_TOPIC4) == 0) {
     /* Topic used to manually reset Days Running */
    //daysRunning = atoi((char *)payload);
    daysRunning = atoi(message);
    saveDaysRunning();
    Serial.println(F(" Days Running Updated"));
    publishMQTT(MQTT_PUB_HELLO, "Days Running Updated");
  } else if (strcmp(topic, MQTT_SUB_TOPIC5) == 0) {
    // Topic used to change car counter timeout  
    //gateCounterTimeout = atoi((char *)payload);
    gateCounterTimeout = atoi(message);
    Serial.println(F(" Gate Counter Alarm Timer Updated"));
    publishMQTT(MQTT_PUB_HELLO, "Gate Counter Timeout Updated");
  }  else if (strcmp(topic, MQTT_SUB_CARMS) == 0) {
    // Topic used to change carDetectMS  
    //gateCounterTimeout = atoi((char *)payload);
    carDetectMS = atoi(message);
    Serial.println(F(" Gate Counter carDetectMS"));
    publishMQTT(MQTT_PUB_HELLO, "Gate Counter carDetectMS Updated");
  } else if (strcmp(topic, MQTT_SUB_LOGGING) == 0) {
        if (strcmp(message, "1") == 0) {
            loggingEnabled = true;
            Serial.println("Sensor logging ENABLED.");
            publishMQTT(MQTT_DEBUG_LOG, "Sensor logging enabled.");
        } else if (strcmp(message, "0") == 0) {
            loggingEnabled = false;
            Serial.println("Sensor logging DISABLED.");
            publishMQTT(MQTT_DEBUG_LOG, "Sensor logging disabled.");
        }
    }
}
/***** END OF CALLBACK TOPICS *****/

/***** IDLE STUFF  *****/
void logSensorStates() {
    if (!loggingEnabled) return; // Do not log if logging is disabled

    static unsigned long lastLogTime = 0; // Prevent excessive logging
    unsigned long currentTime = millis();

    if (currentTime - lastLogTime >= 100) { // Log every 100 ms
        lastLogTime = currentTime;

        // Read sensor states
        beamSensorState = digitalRead(beamSensorPin);
        magSensorState = !digitalRead(magSensorPin); // Active high

        // Log to Serial Monitor (or save to SD card)
        Serial.printf("%lu,%d,%d\n", currentTime, beamSensorState, magSensorState);

        // OPTIONAL: Log to CSV file on SD card
        File logFile = SD.open(fileName10, FILE_APPEND);
        if (logFile) {
            logFile.printf("%lu,%d,%d\n", currentTime, beamSensorState, magSensorState);
            logFile.close();
        } else {
            Serial.println("Failed to open sensorLog.csv");
        }
    }
}

// Average Temperature each hour
void averageHourlyTemp() {
    static int lastPublishedHour = -1;     // Tracks the last hour when data was published
    static int tempReadingsCount = 0;      // Number of valid temperature readings
    static float tempReadingsSum = 0.0;    // Sum of valid temperature readings
    
    // Get the current time
    DateTime now = rtc.now();
    int nowHour = now.hour();

    // Check if the hour has changed
    if (nowHour != lastPublishedHour) {
        // Publish the average for the completed hour
        if (tempReadingsCount > 0 && lastPublishedHour >= 0) {
            hourlyTemp[lastPublishedHour] = tempReadingsSum / tempReadingsCount;

            // Publish to MQTT
            char topic[50];
            snprintf(topic, sizeof(topic), "%s/hourly/%02d", MQTT_PUB_TEMP, lastPublishedHour);
            publishMQTT(topic, String(hourlyTemp[lastPublishedHour], 1)); // Publish with 1 decimal place

            // Log the published temperature
            Serial.printf("Hour %02d average temperature published: %.1f°F\n", lastPublishedHour, hourlyTemp[lastPublishedHour]);
            //publishDebugLog("Hourly average temperature published: " + String(hourlyTemp[lastPublishedHour], 1));
        }

        // Reset for the new hour
        lastPublishedHour = nowHour;
        tempReadingsSum = 0.0;
        tempReadingsCount = 0;
    }

    // Add the latest temperature reading if valid
    if (tempF != -999) { // Ensure only valid readings are processed
        tempReadingsSum += tempF;
        tempReadingsCount++;
        //publishDebugLog("Temperature added for averaging: " + String(tempF));
    }
}

// Car Counted, increment the counter by 1 and append to the Exitlog.csv log file on the SD card
void countTheCar() {
    DateTime now = rtc.now();
    Serial.print(now.toString(buf2));
    Serial.print(", Time to pass = ");
    Serial.println(timeToPassMS);
    //Serial.print(", ");
    //Serial.print(String("DateTime::TIMESTAMP_FULL:\t")+now.timestamp(DateTime::TIMESTAMP_FULL));
    //Serial.print(",1,"); 
    totalDailyCars ++;
    // Increment hourly car count
    int currentHour = now.hour();
    hourlyCount[currentHour]++;
    saveDailyTotal(); // Update Daily Total on SD Card to retain numbers with reboot
    saveHourlyCounts();
    // Construct the MQTT topic dynamically
    char topic[60];
    snprintf(topic, sizeof(topic), "%s/hour%02d", MQTT_PUB_CARS_HOURLY, currentHour);
    // Publish current hour's data to MQTT
    publishMQTT(topic, String(hourlyCount[currentHour]));


  // increase Show Count only when show is open
  if (showTime == true) {
    totalShowCars ++;  // increase Show Count only when show is open
    saveShowTotal(); // update show total count in event of power failure during show hours
  }
  inParkCars=carCounterCars-totalDailyCars;
  // open file for writing Car Data
  myFile = SD.open(fileName6, FILE_APPEND);
  if (myFile) {
    myFile.print(now.toString(buf2));
    myFile.print(", ");
    myFile.print (timeToPassMS) ; 
    myFile.print(", ");
    myFile.print (totalDailyCars) ; 
    myFile.print(", ");
    myFile.print(inParkCars);
    myFile.print(", ");
    myFile.print(tempF);
    myFile.print(" , ");
    myFile.println(magSensorTripTime); //Prints millis when car is detected
    myFile.close();
    /*
    Serial.print(F("Car Saved to SD Card. Car Number = "));
    Serial.print(totalDailyCars);
    Serial.print(F(" Cars in Park = "));
    Serial.println(inParkCars);  
    */
    publishMQTT(MQTT_PUB_HELLO, "Gate Counter Working");
    publishMQTT(MQTT_PUB_TEMP, String(tempF));
    publishMQTT(MQTT_PUB_TIME, now.toString(buf2));
    publishMQTT(MQTT_PUB_EXIT_CARS, String(totalDailyCars));
    publishMQTT(MQTT_PUB_INPARK_CARS, String(inParkCars));
    publishMQTT(MQTT_PUB_BEAM_SENSOR_STATE, String(beamSensorState));
    publishMQTT(MQTT_PUB_MAG_SENSOR_STATE, String(magSensorState));
    publishMQTT(MQTT_PUB_TTP, String(timeToPassMS));
    //snprintf (msg, MSG_BUFFER_SIZE, "Car #%ld,", totalDailyCars);
    //Serial.print("Publish message: ");
    //Serial.println(msg);
    //mqtt_client.publish("msbGateCount", msg);
    //}
  } else {
    Serial.print(F("SD Card: Cannot open the file: "));
    Serial.println(fileName6);
  }
} 

  /* 24/10/14 - Both beams are normally open. Optocoupler reads HIGH when sensors are NOT tripped
  changed code to read inverse of pin. Changing pinmode from pullup or pulldown made no difference 
  Continually Read state of sensors. REVISED 12/12/24 with new Through Beam Sensor 
  When the magSensor drops LOW during the IDLE state and beamSensor is also LOW, reset magSensorWasTriggered
  to prepare for a new car.*/
void detectCar() {
    // Read sensor states
    magSensorState = !digitalRead(magSensorPin);  // Assuming active high
    beamSensorState = digitalRead(beamSensorPin); // Assuming active low (Normally closed = 1)

    // Declare static variables for state tracking
    static unsigned long beamSensorHighTime = 0;  // Time when BeamSensor goes HIGH
    static unsigned long magSensorTripTime = 0;   // Time when MagSensor last triggered
    static unsigned long lastCarPassTime = 0;     // Time of the last car detection
    static bool magSensorTriggered = false;       // Tracks if MagSensor was triggered
    static bool systemReadyLogged = false;        // Tracks if "System ready" has been logged
    static bool alarmTriggered = false;            // Tracks if BeamSensor alarm has been triggered
    static unsigned long lastAlarmPublishTime = 0; // Time of the last alarm message publish
    unsigned long currentMillis = millis();

    // Publish BeamSensor state changes
    if (beamSensorState != lastbeamSensorState) {
        lastbeamSensorState = beamSensorState;
        publishMQTT(MQTT_PUB_BEAM_SENSOR_STATE, String(beamSensorState));
    }

    // Publish MagSensor state changes
    if (magSensorState != lastmagSensorState) {
        lastmagSensorState = magSensorState;
        publishMQTT(MQTT_PUB_MAG_SENSOR_STATE, String(magSensorState));
    }

    // Detect MagSensor pre-trigger and reset if BeamSensor doesn't activate
    if (magSensorState == 1 && !magSensorTriggered) {
        magSensorTripTime = millis();
        magSensorTriggered = true;
        publishMQTT(MQTT_COUNTER_LOG,"MagSensor triggered (pre-beam check)!");
    }

    if (magSensorTriggered && (millis() - magSensorTripTime > 750) && beamSensorHighTime == 0) {
        // Reset MagSensor if BeamSensor hasn't activated within 750ms
        magSensorTriggered = false;
        publishMQTT(MQTT_COUNTER_LOG,"MagSensor reset due to no BeamSensor activation.");
    }

    // Detect BeamSensor state changes
    if (beamSensorState == 1 && beamSensorHighTime == 0) {
        // Beam rising edge detected
        beamSensorHighTime = millis();
        magSensorTriggered = false; // Reset mag sensor trigger for this car
        publishMQTT(MQTT_COUNTER_LOG,"BeamSensor HIGH detected!");
        systemReadyLogged = false;  // Reset "System ready" log flag
    }

    if (beamSensorState == 0 && beamSensorHighTime > 0) {
        // Beam falling edge detected
        unsigned long beamHighDuration = millis() - beamSensorHighTime;
        publishMQTT(MQTT_COUNTER_LOG,"BeamSensor LOW detected. Duration: " + String(beamHighDuration) + " ms");
        publishMQTT(MQTT_PUB_BEAMHIGH_MS, String(beamHighDuration)); // for statistics

        // Check car conditions: Beam active for car not person or MagSensor triggered
        if (beamHighDuration >= carDetectMS || magSensorTriggered) {
           unsigned long currentCarPassTime = millis();
           timeToPassMS = currentCarPassTime-beamSensorHighTime; 
           
           // Calculate time between cars if applicable
            if (lastCarPassTime > 0) {
                unsigned long timeBetweenCars = currentCarPassTime - lastCarPassTime;
                publishMQTT(MQTT_PUB_BETWEENCARS_MS, String(timeBetweenCars));
            }

           lastCarPassTime = currentCarPassTime; // Update last car pass time
           countTheCar();  // Count the car
           publishMQTT(MQTT_COUNTER_LOG,"Car confirmed and counted!");
           publishMQTT(MQTT_PUB_TTP, String(timeToPassMS));
        } else {
            publishMQTT(MQTT_COUNTER_LOG,"No car detected (Beam duration too short or no MagSensor trigger). ");
        }

        // Reset beam timing and mag sensor state
        beamSensorHighTime = 0;
        magSensorTriggered = false; // Ensure mag sensor is reset after car passes
    }

    // Detect MagSensor activations during BeamSensor HIGH
    if (magSensorState == 1 && !magSensorTriggered && beamSensorHighTime > 0) {
        // MagSensor triggered for the first time during BeamSensor HIGH
        magSensorTripTime = millis();
        magSensorTriggered = true;
        publishMQTT(MQTT_COUNTER_LOG,"MagSensor triggered during BeamSensor HIGH!");
    }

    // Log "System ready for next car" only once
    if (beamSensorHighTime == 0 && !magSensorTriggered && !systemReadyLogged) {
        publishMQTT(MQTT_COUNTER_LOG,"System ready for next car.");
        systemReadyLogged = true;
    }

    // Beam Sensor Alarm
    if (beamSensorState == 1) {
        if (beamSensorHighTime == 0) {
            beamSensorHighTime = currentMillis; // Start timing
        } else if (currentMillis - beamSensorHighTime >= 180000) {
            // If HIGH for over 3 minutes
            if (!alarmTriggered) {
                alarmTriggered = true;
                publishMQTT(MQTT_PUB_HELLO, "ALARM: BeamSensor stuck HIGH for over 3 minutes!");
                lastAlarmPublishTime = currentMillis; // Record the time of the alarm
            }
            // Republish the alarm message every 5 minutes
            else if (currentMillis - lastAlarmPublishTime >= 300000) {
                publishMQTT(MQTT_PUB_HELLO, "ALARM: BeamSensor still stuck HIGH.");
                lastAlarmPublishTime = currentMillis; // Update the time of the last publish
            }
        }
    } else {
        // BeamSensor is LOW, reset timing and clear the alarm if it was triggered
        if (alarmTriggered) {
            publishMQTT(MQTT_PUB_HELLO, "BeamSensor alarm cleared: Sensor is LOW.");
            alarmTriggered = false;
        }
        beamSensorHighTime = 0;
    }

}
// END CAR DETECTION

// CHECK AND CREATE FILES on SD Card and WRITE HEADERS if Needed
void checkAndCreateFile(const String &fileName, const String &header = "") {
    if (!SD.exists(fileName)) {
        Serial.printf("%s not found. Creating...\n", fileName.c_str());
        if (fileName.endsWith("/")) { // Create directory if it ends with '/'
            if (!SD.mkdir(fileName)) {
                Serial.printf("Failed to create directory %s\n", fileName.c_str());
                while (1);
            } else {
                Serial.printf("Directory %s created successfully\n", fileName.c_str());
            }
        } else { // Create file if not a directory
            File file = SD.open(fileName, FILE_WRITE);
            if (!file) {
                Serial.printf("Failed to create file %s\n", fileName.c_str());
                while (1);
            } else {
                if (!header.isEmpty()) {
                    file.print(header);
                }
                file.close();
                Serial.printf("File %s created successfully\n", fileName.c_str());
            }
        }
    }
}

void createAndInitializeHourlyFile(const String &fileName) {
    if (!SD.exists(fileName)) {
        Serial.printf("%s not found. Creating...\n", fileName.c_str());
        File file = SD.open(fileName, FILE_WRITE);
        if (!file) {
            Serial.printf("Failed to create file %s\n", fileName.c_str());
            return;
        }

        // Write the header
        file.println("Date,Hr 00,Hr 01,Hr 02,Hr 03,Hr 04,Hr 05,Hr 06,Hr 07,Hr 08,Hr 09,Hr 10,Hr 11,Hr 12,Hr 13,Hr 14,Hr 15,Hr 16,Hr 17,Hr 18,Hr 19,Hr 20,Hr 21,Hr 22,Hr 23");

        // Add an initial row for the current day
        DateTime now = rtc.now();
        char dateBuffer[13];
        snprintf(dateBuffer, sizeof(dateBuffer), "%04d-%02d-%02d", now.year(), now.month(), now.day());
        file.print(dateBuffer); // Write the current date

        // Initialize 24 hourly counts to 0
        for (int i = 0; i < 24; i++) {
            file.print(",0");
        }
        file.println(); // Move to the next line

        file.close();
        Serial.println("Hourly file created and initialized for the current day.");
    } else {
        Serial.printf("File %s already exists. No initialization performed.\n", fileName.c_str());
    }
}

/** Initilaize microSD card */
void initSDCard() {
  if(!SD.begin(PIN_SPI_CS)) {
    Serial.println("Card Mount Failed");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,line1);
    display.println("Check SD Card");
    display.display();
    while (1); // stop the program and check SD Card
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);

  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.println(F("SD CARD INITIALIZED."));
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,line1);
  display.printf("SD Card Size: %lluMB\n", cardSize);
  display.display();
}

void readTempandRH() {
    static unsigned long lastDHTReadMillis = 0;    // Last time temperature was read
    static unsigned long lastDHTPrintMillis = 0;   // Last time temperature was printed
    const unsigned long dhtReadInterval = 10000;  // 10 seconds interval for reading temp
    const unsigned long dhtPrintInterval = 600000; // 10 minutes interval for printing temp
    static bool tempOutOfRangeReported = false;

    unsigned long currentMillis = millis();

    // Check if it's time to read the sensor
    if (currentMillis - lastDHTReadMillis >= dhtReadInterval) {
        lastDHTReadMillis = currentMillis;

        // Read temperature and humidity
        humidity = dht.readHumidity();
        tempF = dht.readTemperature(true); // Read Fahrenheit directly

        // Check if the readings are valid
        if (isnan(tempF) || isnan(humidity)) {
            Serial.println("Failed to read from DHT sensor!");
            publishDebugLog("DHT sensor reading failed.");
            tempF = -999;  // Use a sentinel value to indicate failure
            humidity = -999;
            return; // Exit function if the readings are invalid
        }
        // Check for temperature out of range
        if (tempF < -40 || tempF > 120) {
            if (!tempOutOfRangeReported) {
                // Publish only if not already reported
                Serial.println("Temperature out of range!");
                publishDebugLog("DHT temperature out of range: " + String(tempF));
                tempOutOfRangeReported = true; // Set flag to prevent duplicate reporting
            }
            tempF = -999; // Set to sentinel value for out-of-range condition
        } else {
            // Reset the flag if temperature is back in range
            if (tempOutOfRangeReported) {
                Serial.println("Temperature back in range.");
                tempOutOfRangeReported = false;
            }

            // Publish the temperature and humidity as JSON to MQTT
            char jsonPayload[100];
            snprintf(jsonPayload, sizeof(jsonPayload), "{\"tempF\": %.1f, \"humidity\": %.1f}", tempF, humidity);
            publishMQTT(MQTT_PUB_TEMP, String(jsonPayload));
            //publishDebugLog("Temperature and humidity published: " + String(jsonPayload));

            // Forward valid readings to the hourly average system
            averageHourlyTemp(); // Ensure the reading is processed for summaries
        }

        // Check if it's time to print the readings
        if (currentMillis - lastDHTPrintMillis >= dhtPrintInterval) {
            lastDHTPrintMillis = currentMillis;

            // Print temperature and humidity readings
            if (tempF != -999 && humidity != -999) {
                Serial.printf("Temperature: %.1f °F, Humidity: %.1f %%\n", tempF, humidity);
            } else {
                Serial.println("Temperature/Humidity data invalid. Check sensor.");
            }
        }
    }   
}

/** Resets the hourly count array at midnight */
void resetHourlyCounts() {
    for (int i = 0; i < 24; i++) {
        hourlyCount[i] = 0; // Reset the hourly counts
    }
    // Log the reset
    Serial.println("Hourly counts have been reset.");
    publishMQTT(MQTT_DEBUG_LOG, "Hourly counts reset for the new day.");
}

/* Resets counts at Start of Show and Midnight */
void timeTriggeredEvents() {
    DateTime now = rtc.now();

    // Reset hourly counts at midnight
    if (now.hour() == 23 && now.minute() == 59 && !flagMidnightReset) { 
        resetHourlyCounts();  // Reset array for collecting hourly car counts      
        totalDailyCars = 0;   // Reset total daily cars to 0 at midnight
        saveDailyTotal();
        publishMQTT(MQTT_DEBUG_LOG, "Total cars reset at Midnight");
        flagMidnightReset = true;
    }
    
    // Increment days running only if not Christmas Eve
    if (now.day() != lastDayOfMonth) {
        if (!(now.month() == 12 && now.day() == 24) && !flagDaysRunningReset) {
            daysRunning++;
            saveDaysRunning();
            publishMQTT(MQTT_DEBUG_LOG, "Days running: " + String(daysRunning));
        }
        dayOfMonth = now.day(); // Update day of month
        saveDayOfMonth(); // Save new day of month
        getDayOfMonth(); // Get Day of month
        publishMQTT(MQTT_DEBUG_LOG, "Day of month: " + String(dayOfMonth));
        flagDaysRunningReset = true;
    }

    // Reset total daily cars for show to 0 at 5:10 PM
    if (now.hour() == 17 && now.minute() == 10 && !flagDailyShowStartReset) {
        totalDailyCars = 0;
        saveDailyTotal();
        publishMQTT(MQTT_DEBUG_LOG, "Total Exit Cars Reset at 5:10 PM");
        flagDailyShowStartReset = true;
    }

    // Save daily summary at 9:20 PM
    if (now.hour() == 21 && now.minute() == 20 && !flagDailyShowSummarySaved) {
        saveDailyShowSummary();
        publishMQTT(MQTT_DEBUG_LOG, "Daily Show Summary Saved");
        flagDailyShowSummarySaved = true;
    }

    // 5 minute Event Timer
    if (now.minute() %5 == 0  && !flagHourlyReset) {
        //saveHourlyCounts(); // Saves hourly counts
        flagHourlyReset = true;
        // Add debug message with counts
        char debugMessage[100]; // Increase size if necessary
        int currentHour = now.hour(); // Get the current hour
        snprintf(debugMessage, sizeof(debugMessage), "Hourly data saved for hour %02d. Current count: %d.", 
                currentHour, hourlyCount[currentHour]);
        // Publish the debug message
        publishMQTT(MQTT_DEBUG_LOG, debugMessage);
        Serial.println(debugMessage); // Print to serial for debugging
    }

    // reset 5 Min Event Timer
    if (now.minute() %5 != 0) {
        flagHourlyReset = false;
    }


    // Reset flags for the next day at 12:01:01 AM
    if (now.hour() == 0 && now.minute() == 1 && now.second() == 1 && !resetFlagsOnce) { 
        flagDaysRunningReset = false;
        flagMidnightReset = false;
        flagDailyShowStartReset = false;
        flagDailySummarySaved = false;
        flagDailyShowSummarySaved = false;
        flagHourlyReset = false;
        publishMQTT(MQTT_DEBUG_LOG, "Run once flags reset for new day");
        resetFlagsOnce = true; // Prevent further execution within the same day
    }

    // Reset the `resetFlagsOnce` at 12:02:00 AM to allow it to run the next day
    if (now.hour() == 0 && now.minute() == 2 && now.second() == 0) {
        resetFlagsOnce = false; // Allow reset logic to run again the next day
    }
}


// Update OLED Display while running
void updateDisplay() {
    DateTime now = rtc.now();
    //float tempF = ((rtc.getTemperature() * 9 / 5) + 32); // Get temperature in Fahrenheit
    int currentHr24 = now.hour();
    int currentHr12 = currentHr24 > 12 ? currentHr24 - 12 : (currentHr24 == 0 ? 12 : currentHr24);
    const char* ampm = currentHr24 < 12 ? "AM" : "PM";

    // Clear display and set formatting
    display.clearDisplay();
    display.setTextColor(WHITE);
    //display.setFont(&FreeSans12pt7b);

    // Line 1: Date and Day of the Week
    display.setTextSize(1);
    display.setCursor(0, line1);
    display.printf("%s %s %02d, %04d", days[now.dayOfTheWeek()], months[now.month() - 1], now.day(), now.year());

    // Line 2: Time and Temperature
    display.setCursor(0, line2);
    display.printf("%02d:%02d:%02d %s  %d F", currentHr12, now.minute(), now.second(), ampm, (int)tempF);

    // Line 3: Days Running and Show Total
    display.setCursor(0, line3);
    display.printf("Day %d  Total: %d", daysRunning, totalShowCars);

    // Line 4: Total Daily Cars
    display.setTextSize(1);
    display.setCursor(0, line4);
    display.printf("Exiting: %d", totalDailyCars);

    //Line 5: In Park Cars
    display.setTextSize(1);
    display.setCursor(0, line5);
    display.printf("InPark: %d", carCounterCars - totalDailyCars);

    // Write to the display
    display.display();
}

/******  BEGIN SETUP ******/
void setup() {
    Serial.begin(115200);
    ElegantOTA.setAutoReboot(true);
    //ElegantOTA.setFilesystemMode(true);
    Serial.println("Starting Gate Counter...");

  //Initialize Display
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, line1);
    display.println("Initializing...");
    display.display();

// Set the ESP32 hostname
    if (WiFi.setHostname(THIS_MQTT_CLIENT)) {
        Serial.printf("Hostname set to: %s\n", THIS_MQTT_CLIENT);
    } else {
        Serial.println("Failed to set hostname!");
    }

    // Scan WiFi networks
    WiFi.mode(WIFI_STA);
    int n = WiFi.scanNetworks();
    Serial.println("WiFi scan completed.");
    if (n == 0) {
        Serial.println("No networks found.");
    } else {
        Serial.printf("%d networks found:\n", n);
        for (int i = 0; i < n; ++i) {
            Serial.printf("%d: %s (%d dBm) %s\n",
                          i + 1,
                          WiFi.SSID(i).c_str(),
                          WiFi.RSSI(i),
                          WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN" : "SECURE");
        }
    }

    // Add multiple WiFi APs
    wifiMulti.addAP(secret_ssid_AP_1, secret_pass_AP_1);
    wifiMulti.addAP(secret_ssid_AP_2, secret_pass_AP_2);
    wifiMulti.addAP(secret_ssid_AP_3, secret_pass_AP_3);
    wifiMulti.addAP(secret_ssid_AP_4, secret_pass_AP_4);
    wifiMulti.addAP(secret_ssid_AP_5, secret_pass_AP_5);
    setup_wifi();  

    // Initialize MQTT
    //mqtt_client.setServer(mqtt_server, mqtt_port);
    //mqtt_client.setCallback(callback);

    // MQTT Reconnection with login credentials
    MQTTreconnect(); // Ensure MQTT is connected

    //If RTC not present, stop and check battery
    if (! rtc.begin()) {
        Serial.println("Could not find RTC! Check circuit.");
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0,line1);
        display.println("Clock DEAD");
        display.display();
        while (1);
    }

    // Get NTP time from Time Server 
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    SetLocalTime();

    //Set Input Pins
    pinMode(magSensorPin, INPUT_PULLDOWN);
    pinMode(beamSensorPin, INPUT_PULLDOWN);
    //pinMode(DHTPIN, INPUT_PULLUP);

    // Initialize DHT sensor
    dht.begin();
    Serial.println("DHT22 sensor initialized.");
    readTempandRH();
    
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, line3);
    display.println(" Starting");
    display.println("GateCounter");

    Serial.println  ("Initializing Gate Counter");
    Serial.print("Temperature: ");
    Serial.print(tempF);
    Serial.println(" F");
    display.display();
    delay(500); // display Startup

    //Initialize SD Card
    SD.begin(PIN_SPI_CS);
    initSDCard();  // Initialize SD card and ready for Read/Write

    // Check and create Required Data files
    checkAndCreateFile(fileName1);
    checkAndCreateFile(fileName2);
    checkAndCreateFile(fileName3);
    checkAndCreateFile(fileName4);
    //checkAndCreateFile(fileName5, "Date,Hour-17,Hour-18,Hour-19,Hour-20,Hour-21,Total,Temp");
    checkAndCreateFile(fileName6, "Date Time,TimeToPass,Car#,Cars In Park,Temp,Car Detected Millis");
    checkAndCreateFile(fileName7, "Date,DaysRunning,Before5,6PM,7PM,8PM,9PM,ShowTotal,DailyAvgTemp");
    checkAndCreateFile(fileName8);
    checkAndCreateFile(fileName9);
    checkAndCreateFile(fileName10, "ms,Beam,mag");

    // Required Hourly Data Files
    createAndInitializeHourlyFile(fileName5);

    // Initialize Server
    setupServer();

    //on reboot, get totals saved on SD Card
    getSavedValuesOnReboot();  // Update/reset counts based on reboot day

    // Setup MDNS
    if (!MDNS.begin(THIS_MQTT_CLIENT)) {
      Serial.println("Error starting mDNS");
      return;
    }
    delay(1000);
    start_MqttMillis = millis();
} /***** END SETUP ******/

void loop() {  
    DateTime now = rtc.now();

    currentTimeMinute = now.hour()*60 + now.minute(); // convert time to minutes since midnight

    showTime = (currentTimeMinute >= showStartMin && currentTimeMinute <= showEndMin); // show is running and save counts

    readTempandRH();          // Get Temperature and Humidity

    ElegantOTA.loop();        // Keep OTA Updates Alive

    updateDisplay();          // Update the display

    checkWiFiConnection();    // Check and maintain WiFi connection

    timeTriggeredEvents();    // Various functions/saves/resets based on time of day

    detectCar();              // Detect cars

    //Added to kepp mqtt connection alive and periodically publish select values
    if ((millis() - start_MqttMillis) > (mqttKeepAlive * 1000)) {
        KeepMqttAlive();
    }
} 
/***** Repeat Loop *****/