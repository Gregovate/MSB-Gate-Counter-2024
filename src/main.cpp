/*
Gate Counter by Greg Liebig gliebig@sheboyganlights.org
Initial Build 12/5/2023 12:15 pm

Changelog
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
Changed time format YYYY-MM-DD hh:mm:ss 12/13/23
10/10/24 
10/15/24
#define FWVersion "24.10.17.2" 
Fixed Pin problem. Beam & mag sensor swapped causing the problems
Purpose: suppliments Car Counter to improve traffic control and determine park capacity
Counts vehicles as they exit the park
Connects to WiFi and updates RTC on Boot
Uses an Optocoupler to read burried vehicle sensor for Ghost Controls Gate operating at 12V
DOIT DevKit V1 ESP32 with built-in WiFi & Bluetooth
SPI Pins
D5 - CS
D18 - CLK
D19 - MISO
D23 - MOSI
*/

#include <Arduino.h>
#include <Wire.h>
#include <PubSubClient.h>
//#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RTClib.h"
#include "NTPClient.h"
//#include <WiFiClientSecure.h>
#include <WiFiMulti.h>
#include "secrets.h"
#include "time.h"
#include "FS.h"
#include "SD.h"
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

// ******************** CONSTANTS *******************
#define magSensorPin 32 // Pin for Magnotometer Sensor
#define beamSensorPin 33  //Pin for Reflective Scensor
#define PIN_SPI_CS 5 // SD Card CS GPIO5
// #define MQTT_KEEPALIVE 30 //removed 10/16/24
#define FWVersion "24.11.8.1" // Firmware Version
#define OTA_Title "Gate Counter" // OTA Title
unsigned int carDetectMillis = 500; // minimum millis for beamSensor to be broken needed to detect a car
unsigned int showStartTime = 17*60 + 10; // Show (counting) starts at 5:10 pm
unsigned int showEndTime =  21*60 + 20;  // Show (counting) ends at 9:20 pm 
// **************************************************

AsyncWebServer server(80);

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
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

//#include <DS3231.h>
RTC_DS3231 rtc;

//Create Multiple WIFI Object

WiFiMulti wifiMulti;
//WiFiClientSecure espGateCounter;
WiFiClient espGateCounter;
PubSubClient mqtt_client(espGateCounter);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (500)
char msg[MSG_BUFFER_SIZE];
int value = 0;

char mqtt_server[] = mqtt_Server;
char mqtt_username[] = mqtt_UserName;
char mqtt_password[] = mqtt_Password;
const int mqtt_port = mqtt_Port;
bool mqtt_connected = false;
bool wifi_connected = false;
bool showTime = false;
int wifi_connect_attempts = 5;

/***** MQTT TOPIC DEFINITIONS *****/
#define THIS_MQTT_CLIENT "espGateCounter" // Look at line 90 and set variable for WiFi Client secure & PubSubCLient 12/23/23
int mqttKeepAlive = 30; // publish temp every x seconds to keep MQTT client connected
// Publishing Topics 
char topic[60];
char topicBase[60];
#define topic_base_path  "msb/traffic/GateCounter"
#define MQTT_PUB_TOPIC0 "msb/traffic/GateCounter/hello"
#define MQTT_PUB_TOPIC1 "msb/traffic/GateCounter/temp"
#define MQTT_PUB_TOPIC2 "msb/traffic/GateCounter/time"
#define MQTT_PUB_TOPIC3 "msb/traffic/GateCounter/count"
#define MQTT_PUB_TOPIC4 "msb/traffic/GateCounter/inParkCars"
#define MQTT_PUB_TOPIC5 "msb/traffic/GateCounter/hour18"
#define MQTT_PUB_TOPIC6 "msb/traffic/GateCounter/hour19"
#define MQTT_PUB_TOPIC7 "msb/traffic/GateCounter/hou20"
#define MQTT_PUB_TOPIC8 "msb/traffic/GateCounter/hour21"
#define MQTT_PUB_TOPIC9 "msb/traffic/GateCounter/DayTot"
#define MQTT_PUB_TOPIC10 "msb/traffic/GateCounter/ShoTot"
#define MQTT_PUB_TOPIC11 "msb/traffic/GateCounter/debug/timeout"
#define MQTT_PUB_TOPIC12 "msb/traffic/GateCounter/debug/beamSensorState"

// Subscribing Topics (to reset values)
#define MQTT_SUB_TOPIC0  "msb/traffic/CarCounter/DayTot"
#define MQTT_SUB_TOPIC1  "msb/traffic/GateCounter/resetcount"


//const uint32_t connectTimeoutMs = 10000;
uint16_t connectTimeOutPerAP=5000;
const char* ampm ="AM";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;
int16_t tempF;

char buf2[25] = "YYYY-MM-DD hh:mm:ss"; // time car detected
char buf3[25] = "YYYY-MM-DD hh:mm:ss"; // time bounce detected
int hourArray[24]; // used to store hourly totals

/***** GLOBAL VARIABLES *****/
unsigned int currentDay;
unsigned int currentHr12;
unsigned int currentHr24;
unsigned int currentMin;
unsigned int currentSec;
unsigned int lastCalDay;
unsigned int daysRunning;
unsigned int currentTimeMinute; // for converting clock time hh:mm to clock time mm

int totalDailyCars;
int totalShowCars;
int inParkCars; // cars in park
int carCounterCars; // Counts from Car Counter
int carsHr18 =0; // total cars hour 18 (6:00 pm)
int carsHr19 =0; // total cars hour 19 (7:00 pm)
int carsHr20 =0; // total cars hour 20 (8:00 pm)
int carsHr21 =0; // total cars hour 21 (9:20 pm)
int magSensorState; /* Store state of Mag Sensor*/
int lastmagSensorState; /* Store Last State of mag Sensor */
int beamSensorState; /* Store state of Beam Sensor */
int lastbeamSensorState; /* Store Last State of Beam Sensor */

int carPresentFlag = 0;  // used to flag when car is in the detection zone
int NoCarFlag = 0;  // used to clear car in detection zone. May not be necessary

/***** TIMER VARIABLES *****/
unsigned long TimeToPassMillis; // time car is in detection Zone
unsigned long beamSensorTripTime; // capture time when beamSensor goes HIGH
unsigned long magSensorTripTime; //  capture time when magSensor goes HIGH
unsigned long beamSensorBounceTime; // capture time if beam sensor bounces
unsigned long carDetectedMillis;  // Grab the ime when sensor 1st trips
unsigned long wifi_connectioncheckMillis = 5000; // check for connection every 5 sec
unsigned long mqtt_connectionCheckMillis = 20000; // check for connection
unsigned long start_MqttMillis; // for Keep Alive Timer
unsigned long start_WiFiMillis; // for keep Alive Timer
unsigned long currentMillis; // loop timer



// **********FILE NAMES FOR SD CARD *********
File myFile; //used to write files to SD Card
const String fileName1 = "/DailyTot.txt"; // /DailyTot.txt file to store daily counts in the event of a Failure
const String fileName2 = "/ShowTot.txt";  // /ShowTot.txt file to store season total counts
const String fileName3 = "/CalDay.txt"; // /CalDay.txt file to store current day number
const String fileName4 = "/RunDays.txt"; // /unDays.txt file to store days since open
const String fileName5 = "/GateSummary.csv"; // /GateSummary.csv Stores Daily Totals by Hour and total
const String fileName6 = "/GateLog.csv"; // GateLog.csv file to store all car counts for season (was MASTER.CSV)
const String fileName7 = "/SensorBounces.csv"; // /SensorBounces.csv file to store all car counts for season (was MASTER.CSV)

/***** Used to make display Pretty *****/
char days[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
int dayHour[24]; // Array for Daily total cars per hour
int showHour[5]; // Array for Show total cars per hour starting at 4:55 pm and ending at 9:10 pm 18, 19, 20, 21

/************* DISPLAY SIZE ************/
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire, -1);
/*Line Numbers used for Display*/
int line1 =0;
int line2 =9;
int line3 = 19;
int line4 = 30;
int line5 = 42;
int line6 = 50;
int line7 = 53;


void setup_wifi() {
    Serial.println("Connecting to WiFi");
    display.println("Connecting to WiFi..");
    display.display();
    while(wifiMulti.run(connectTimeOutPerAP) != WL_CONNECTED) {
      //Serial.print(".");
    }
    Serial.println("Connected to the WiFi network");
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.display();
   // print the SSID of the network you're attached to:
    display.setCursor(0, line1);
    display.print("SSID: ");
    display.println(WiFi.SSID());

    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP: ");
    Serial.println(ip);
    display.setCursor(0, line2);
    display.print("IP: ");
    display.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
    display.setCursor(0, line3);
    display.print("signal: ");
    display.print(rssi);
    display.println(" dBm");
    display.display();
 
  // Elegant OTA
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is The Gate Counter.");
  });

  // You can also enable authentication by uncommenting the below line.
  // ElegantOTA.setAuth("admin", "password");

  ElegantOTA.setID(THIS_MQTT_CLIENT);  // Set Hardware ID
  ElegantOTA.setFWVersion(FWVersion);   // Set Firmware Version
  ElegantOTA.setTitle(OTA_Title);  // Set OTA Webpage Title
  //ElegantOTA.setFilesystemMode(true);  // added 10.16.24.4

  ElegantOTA.begin(&server);    // Start ElegantOTA

  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);


  server.begin();
  Serial.println("HTTP server started");

  delay(5000);
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  mqttKeepAlive = millis(); // reset keep alive timer when message received
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  payload[length] = '\0';

   /* Receive MQTT message with updated CarCounter totals*/
  if (strcmp(topic, MQTT_SUB_TOPIC0) == 0) {
    carCounterCars = atoi((char *)payload);
    inParkCars=carCounterCars-totalDailyCars; // recalculate cars in park
  }
  
  /* Topic used to manually reset gate total cars */
  if (strcmp(topic, MQTT_SUB_TOPIC1) == 0){
    totalDailyCars = atoi((char *)payload);
//    Serial.println(" Gate Counter Updated");
  }
  //  Serial.println(carCountCars);
  Serial.println();
}

void MQTTreconnect()
{
  // Loop until we’re reconnected
  while (!mqtt_client.connected())
  {
    Serial.print("Attempting MQTT connection… ");
    String clientId = THIS_MQTT_CLIENT;
    // Attempt to connect
    if (mqtt_client.connect(clientId.c_str(), mqtt_username, mqtt_password))
    {
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,line5);
      display.println("MQTT Connect");
      display.display();     Serial.println("connected!");
      Serial.println("Waiting for Car");
      // Once connected, publish an announcement…
      mqtt_client.publish(MQTT_PUB_TOPIC0, "Hello from Gate Counter!");
      mqtt_client.publish(MQTT_PUB_TOPIC1, String(tempF).c_str());
      // … and resubscribe
      mqtt_client.subscribe(MQTT_PUB_TOPIC0);
    } 
    else
    {
      Serial.print("failed, rc = ");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,line4);
      display.println("MQTT Error");
      display.display();
    }
  }  // END While
  mqtt_client.subscribe(MQTT_SUB_TOPIC0);
  mqtt_client.subscribe(MQTT_SUB_TOPIC1);
}

void SetLocalTime()
  {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo))
    {
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

// =========== GET SAVED SETUP FROM SD CARD ==========
void getDailyTotal()   // open DAILYTOT.txt to get initial dailyTotal value
{
   myFile = SD.open(fileName1,FILE_READ);
   if (myFile)
   {
     while (myFile.available()) {
     totalDailyCars = myFile.parseInt(); // read total
     Serial.print(" Daily cars from file = ");
     Serial.println(totalDailyCars);
    }
  myFile.close();
  }
  else
  {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName1);
  }
}

void getShowTotal()     // open ShowTot.txt to get totalCars for season
 {
    myFile = SD.open(fileName2,FILE_READ);
    if (myFile)
    {
      while (myFile.available())
      {
        totalShowCars = myFile.parseInt(); // read total
        Serial.print(" Total cars from file = ");
        Serial.println(totalShowCars);
      }
      myFile.close();
    }
    else
    {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName2);
    }
}

void getCalDay()  // get the last calendar day used for reset daily counts)
 {
    myFile = SD.open(fileName3,FILE_READ);
    if (myFile)
    {
       while (myFile.available()) {
       lastCalDay = myFile.parseInt(); // read day Number
       Serial.print(" Calendar Day = ");
       Serial.println(lastCalDay);
    }
    myFile.close();
    }
    else
    {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName3);
    }
} 

void getDaysRunning()   // Days the show has been running)
{
   myFile = SD.open(fileName4,FILE_READ);
   if (myFile)
   {
      while (myFile.available()) {
      daysRunning = myFile.parseInt(); // read day Number
      Serial.print(" Days Running = ");
      Serial.println(daysRunning);
    }
    myFile.close();
    }
    else
    {
      Serial.print(F("SD Card: Cannot open the file: "));
      Serial.println(fileName4);
    }
} 



/***** UPDATE TOTALS TO SD CARD *****/
void HourlyTotals()
{
  dayHour[currentHr24]= totalDailyCars; //write daily total cars to array each hour
  strcpy (topicBase, topic_base_path);
  strcat (topicBase, "/daily/hour/");
  strcat (topicBase, String(currentHr24).c_str());
  mqtt_client.publish(topicBase, String(totalDailyCars).c_str()); // publish counts
}

void KeepMqttAlive()
{
   mqtt_client.publish(MQTT_PUB_TOPIC1, String(tempF).c_str());
   Serial.println("Keeping MQTT Alive");
   start_MqttMillis = currentMillis;
}

void updateDailyTotal()
{
  myFile = SD.open(fileName1,FILE_WRITE);
  if (myFile)
  {  // check for an open failure
     myFile.print(totalDailyCars);
     myFile.close();
  }
  else
  {
     Serial.print(F("SD Card: Cannot open the file:  DailyTot.txt"));
  } 
}

void updateShowTotal()  /* -----Increment the grand total cars file ----- */
{  
   myFile = SD.open(fileName2,FILE_WRITE);
   if (myFile) 
   {
      myFile.print(totalShowCars);
      myFile.close();
  }
  else
  {
    Serial.print(F("SD Card: Cannot open the file ShowTot.TXT"));
  }
}

void updateCalDay()  /* -----Increment the grand total cars file ----- */
{
   myFile = SD.open(fileName3,FILE_WRITE);
   if (myFile)
   {
      myFile.print(currentDay);
      myFile.close();
   }
   else
   {
      Serial.print(F("SD Card: Cannot open the file ShowTot.TXT"));
   }
}

void updateDaysRunning()
{
  myFile = SD.open(fileName4,FILE_WRITE);
  if (myFile) // check for an open failure
  {
    myFile.print(daysRunning);
    myFile.close();
  }
  else
  {
    Serial.print(F("SD Card: Cannot open the file RunDays.txt"));
  }
}

void updateCarCount(){
        DateTime now = rtc.now();
        Serial.print(now.toString(buf3));
        Serial.print(", Time to pass = ");
        Serial.println(TimeToPassMillis);
        //Serial.print(", ");
        //Serial.print(String("DateTime::TIMESTAMP_FULL:\t")+now.timestamp(DateTime::TIMESTAMP_FULL));
        //Serial.print(",1,"); 
        totalDailyCars ++;
        updateDailyTotal(); // Update Daily Total on SD Card to retain numbers with reboot
        if (showTime == true)
        {
          totalShowCars ++;
          updateShowTotal(); // update show total count in event of power failure during show hours
        }
        inParkCars=carCounterCars-totalDailyCars;
        // open file for writing Car Data
        //HEADER: ("Date Time,Pass Timer,NoCar Timer,Bounces,Car#,Cars In Park,Temp,Last Car Millis, This Car Millis,Bounce Flag,Millis");
        myFile = SD.open(fileName6, FILE_APPEND);
        if (myFile) {
          myFile.print(now.toString(buf3));
          myFile.print(", ");
          myFile.print (TimeToPassMillis) ; 
          myFile.print(", ");
          myFile.print (totalDailyCars) ; 
          myFile.print(", ");
          myFile.print(inParkCars);
          myFile.print(", ");
          myFile.print(tempF);
          myFile.print(" , ");
          myFile.println(carDetectedMillis); //Prints car number being detected
          myFile.close();
              
          Serial.print(F("Car Saved to SD Card. Car Number = "));
          Serial.print(totalDailyCars);
          Serial.print(F(" Cars in Park = "));
          Serial.println(inParkCars);  
          mqtt_client.publish(MQTT_PUB_TOPIC1, String(tempF).c_str());
          mqtt_client.publish(MQTT_PUB_TOPIC2, now.toString(buf3));
          mqtt_client.publish(MQTT_PUB_TOPIC3, String(totalDailyCars).c_str());
          mqtt_client.publish(MQTT_PUB_TOPIC4, String(inParkCars).c_str());
          mqtt_client.publish(MQTT_PUB_TOPIC12, String(beamSensorState).c_str());
          //snprintf (msg, MSG_BUFFER_SIZE, "Car #%ld,", totalDailyCars);
          //Serial.print("Publish message: ");
          //Serial.println(msg);
          //mqtt_client.publish("msbGateCount", msg);
          //}
        } else {
          Serial.print(F("SD Card: Issue encountered while attempting to open the file GateCount.csv"));
        }
}

/***** END OF FILE OPS *****/

void initSDCard()
{
  if(!SD.begin(PIN_SPI_CS)){
    Serial.println("Card Mount Failed");
    //Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
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

  if(cardType == CARD_NONE){
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
  //display.println("SD Card Ready");
  display.printf("SD Card Size: %lluMB\n", cardSize);
  display.display();
}

/******  BEGIN SETUP ******/
void setup() 
{
    Serial.begin(115200);
    ElegantOTA.setAutoReboot(true);

    //Initialize Display
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.display();

    //Initialize SD Card
      initSDCard();
  /*
    if (!SD.begin(PIN_SPI_CS)) {
      Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,line1);
      display.println("Check SD Card");
      display.display();
      while (1); // stop the program and check SD Card
    }

    Serial.println(F("SD CARD INITIALIZED."));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,line1);
    display.println("SD Card Ready");
    display.display();
  */
    //***** Check AND/OR Prep Files for use ******/ 
    if (!SD.exists(fileName1))
    {
      Serial.println(F("DailyTot.txt doesn't exist. Creating file..."));
      // create a new file by opening a new file and immediately close it
      myFile = SD.open(fileName1, FILE_WRITE);
      myFile.close();
      }
      else
      {
        Serial.print(fileName1);
        Serial.println(F(" exists on SD Card."));
      }
    if (!SD.exists(fileName2))
    {
      Serial.println(F("ShowTot.txt doesn't exist. Creating file..."));
      // create a new file by opening a new file and immediately close it
      myFile = SD.open(fileName2, FILE_WRITE);
      myFile.close();
    }
    else
    {
      Serial.print(fileName2);
      Serial.println(F(" exists on SD Card."));
    }

    if (!SD.exists(fileName3))
    {
      Serial.println(F("CalDay.txt doesn't exist. Creating file..."));
      // create a new file by opening a new file and immediately close it
      myFile = SD.open(fileName3, FILE_WRITE);
      myFile.close();
    }
    else
    {
      Serial.print(fileName3);
      Serial.println(F(" exists on SD Card."));
    }

    if (!SD.exists(fileName4))
    {
      Serial.println(F("RunDays.txt doesn't exist. Creating file..."));
      // create a new file by opening a new file and immediately close it
      myFile = SD.open(fileName4, FILE_WRITE);
      myFile.close();
    }
    else
    {
      Serial.print(fileName4);
      Serial.println(F(" exists on SD Card."));
    }
    if (!SD.exists(fileName5))
    {
      Serial.println(F("GateSummary.csv doesn't exist. Creating file..."));
      // create a new file by opening a new file and immediately close it
      myFile = SD.open(fileName5, FILE_WRITE);
      myFile.close();
      // recheck if file is created & write Header
      myFile = SD.open(fileName5, FILE_APPEND);
      myFile.println("Date, Temp, Hour1, Hour2, Hour3, Hour4, Total");
      myFile.close();
      Serial.println(F("Header Written to file DailySummary.csv"));
      myFile.close();
    }
    else
    {
      Serial.print(fileName5);
      Serial.println(F(" exists on SD Card."));
    }

    if (!SD.exists(fileName6))
    {
      myFile = SD.open(fileName6, FILE_WRITE);
      myFile.close();
      myFile = SD.open(fileName6, FILE_APPEND);
      // recheck if file is created write Header
      //HEADER: ("Date Time,Pass Timer,NoCar Timer,Bounces,Car#,Cars In Park,Temp,Last Car Millis, This Car Millis,Bounce Flag,Millis");
      myFile.println("Date Time,Pass Timer,NoCar Timer,Bounces,Car#,Cars In Park,Temp,Last Car Millis, This Car Millis,Bounce Flag,Millis");
      myFile.close();
      Serial.println(F("Header Written to GateCount.csv"));
    } 
    else
    {
      Serial.print(fileName6);
      Serial.println(F(" exists on SD Card."));
    }

    
    if (!SD.exists(fileName7))
    {
      Serial.println(F("SensorBounces.csv doesn't exist. Creating SensorBounces.csv file..."));
      // create a new file by opening a new file and immediately close it
      myFile = SD.open(fileName7, FILE_WRITE);
      myFile.close();
      myFile = SD.open(fileName7, FILE_APPEND);
      //("DateTime\t\t\tPassing Time\tLast High\tDiff\tLow Millis\tLast Low\tDiff\tBounce #\tCurent State\tCar#" )
      myFile.println("DateTime,TimeToPass,Last Beam Low,Beam Low,Diff,Beam State, Bounce#,Car#");
      myFile.close();
      Serial.println(F("Header Written to SensorBounces.csv"));
    }
    else
    {
      Serial.print(fileName7);
      Serial.println(F(" exists on SD Card."));
    }
      
  // List of approved WiFi AP's
    WiFi.mode(WIFI_STA); 
    wifiMulti.addAP(secret_ssid_AP_1,secret_pass_AP_1);
    wifiMulti.addAP(secret_ssid_AP_2,secret_pass_AP_2);
    wifiMulti.addAP(secret_ssid_AP_3,secret_pass_AP_3);
    wifiMulti.addAP(secret_ssid_AP_4,secret_pass_AP_4);
    wifiMulti.addAP(secret_ssid_AP_5,secret_pass_AP_5);
    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0)
    {
      Serial.println("no networks found");
    } 
    else
    {
      Serial.print(n);
      Serial.println(" networks found");
      for (int i = 0; i < n; ++i)
      {
        // Print SSID and RSSI for each network found
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.print(")");
        Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
        delay(10);
      }
    }
    
    setup_wifi();
    mqtt_client.setServer(mqtt_server, mqtt_port);
    mqtt_client.setCallback(callback);

    //If RTC not present, stop and check battery
    if (! rtc.begin())
    {
      Serial.println("Could not find RTC! Check circuit.");
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE);
      display.setCursor(0,line1);
      display.println("Clock DEAD");
      display.display();
      while (1);
    }

    // Get NTP time from Time Server 
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    SetLocalTime();
    
    //Set Input Pin
    pinMode(magSensorPin, INPUT_PULLDOWN);
    pinMode(beamSensorPin, INPUT_PULLDOWN);
  
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0, line4);
    display.print("GATE Count");

    Serial.println  ("Initializing Gate Counter");
    Serial.print("Temperature: ");
    tempF=((rtc.getTemperature()*9/5)+32);
    Serial.print(tempF);
    Serial.println(" F");
    display.display();

    //on reboot, get totals saved on SD Card
    getDailyTotal();  /*Daily total that is updated with every detection*/
    getDaysRunning(); /*Needs to be reset 1st day of show*/
    getCalDay();  /*Saves Calendar Day*/
    getShowTotal();   /*Saves Show Total*/


    if (!MDNS.begin(THIS_MQTT_CLIENT))
    {
      Serial.println("Error starting mDNS");
      return;
    }
    
    delay(3000);
    start_MqttMillis = millis();
} //***** END SETUP ******/

void loop()
{  
   //Required for OTA Programming
   ElegantOTA.loop();
  
   DateTime now = rtc.now();
   tempF=((rtc.getTemperature()*9/5)+32);
   currentMillis = millis(); 
  
   showTime = (currentTimeMinute >= showStartTime && currentTimeMinute <= showEndTime); // show is running and save counts

  /*****IMPORTANT***** Reset Gate Counter at 5:10:00  *****/
  /* Only counting vehicles for show */
  if ((now.hour() == 17) && (now.minute() == 10) && (now.second() == 0))  {
     totalDailyCars = 0;
     updateDailyTotal();
  }    
  
  //Write Totals at 9:20:00 pm. Gate should close at 9 PM. Allow for any cars in line get through
  if ((now.hour() == 21) && (now.minute() == 20) && (now.second() == 0))
  {
    DateTime now = rtc.now();
    tempF = ((rtc.getTemperature()*9/5)+32);
    char buf2[] = "YYYY-MM-DD hh:mm:ss";
    Serial.print(now.toString(buf2));
    Serial.print(", Temp = ");
    Serial.print(tempF);
    Serial.print(", ");
    Serial.print(totalDailyCars) ;  
    // open file for writing Car Data
    myFile = SD.open(fileName5, FILE_APPEND);
    if (myFile) 
    {
      myFile.print(now.toString(buf2));
      myFile.print(", ");
      myFile.print (tempF); 
      myFile.print(", "); 
      myFile.print (carsHr18) ; 
      myFile.print(", ");
      myFile.println(carsHr19);
      myFile.print(", ");
      myFile.println(carsHr20);
      myFile.print(", ");
      myFile.println(carsHr21);
      myFile.print(", ");
      myFile.println(totalDailyCars);
      myFile.close();
      Serial.println(F(" = Daily Summary Recorded SD Card."));
        // Publish Totals
      mqtt_client.publish(MQTT_PUB_TOPIC1, String(tempF).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC2, now.toString(buf2));
      mqtt_client.publish(MQTT_PUB_TOPIC3, String(totalDailyCars).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC4, String(inParkCars).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC5, String(carsHr18).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC6, String(carsHr19).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC7, String(carsHr20).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC8, String(carsHr21).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC9, String(totalDailyCars).c_str());
      mqtt_client.publish(MQTT_PUB_TOPIC10, String(totalShowCars).c_str());
    }
    else
    {
      Serial.print(F("SD Card: Issue encountered while attempting to open the file CarCount.csv"));
    }
  }
  
  /* Reset Counts at Midnight when controller running 24/7 */
  if ((now.hour() == 0) && (now.minute() == 0) && (now.second() == 1))
  {
      currentDay = now.day();
      updateCalDay();
      totalDailyCars = 0;
      updateDailyTotal();
      daysRunning++;
      updateDaysRunning();
  }
  /* OR Reset/Update Counts wwhen Day Changes on reboot getting values from saved data */
  if (now.day() != lastCalDay)
  {
      getCalDay();
      currentDay=now.day();
      updateCalDay();
      totalDailyCars =0;
      updateDailyTotal();
      daysRunning++;
      updateDaysRunning();
  }

  /* non-blocking WiFi and MQTT Connectivity Checks 
  First check if WiFi is connected */
  if (wifiMulti.run() == WL_CONNECTED)
  {
    /* If MQTT is not connected then Attempt MQTT Connection */
    if (!mqtt_client.connected())
    {
      if (currentMillis - start_MqttMillis > mqtt_connectionCheckMillis)
      {
        Serial.print("hour = ");
        Serial.println(currentHr12);
        Serial.println("Attempting MQTT Connection");
        MQTTreconnect();
        start_MqttMillis = currentMillis;
      }   
    } else {
         //keep MQTT client connected when WiFi is connected
         mqtt_client.loop();
    }
  } else
  {
       // If WiFi if lost, then attemp non blocking WiFi Connection
       if ((currentMillis - start_WiFiMillis) > wifi_connectioncheckMillis)
       {
          setup_wifi();
          start_WiFiMillis = currentMillis;
       }
  }    

  /* Display Program Data */

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, line1);

  //  display Day of Week
  display.print(days[now.dayOfTheWeek()]);

  //  Display Date
  display.print(" ");         
  display.print(months[now.month(), DEC +1]);
  display.print(" ");
  display.print(now.day(), DEC);
  display.print(", ");
  display.println(now.year(), DEC);
  
  // Convert 24 hour clock to 12 hours for display purposes
  currentHr24 = now.hour();
  if (currentHr24 <12)  {
    ampm ="AM";
  } else {
    ampm ="PM";
  }
  if (currentHr24 > 12 )  {
    currentHr12 = now.hour() - 12;
  } else {
    currentHr12 = now.hour();
  }

  /***** Display Time add leading 0 to Hours & display Hours *****/
  display.setTextSize(1);
  if (currentHr12 < 10){
    display.setCursor(0, line2);
    display.print("0");
    display.println(currentHr12, DEC);
  }else{
    display.setCursor(0, line2);
    display.println(currentHr12, DEC);
  }
  display.setCursor(14, line2);
  display.println(":");

  /***** Add leading 0 To Mintes & display Minutes *****/
  if (now.minute() < 10)
  {  
    display.setCursor(20, line2);
    display.print("0");
    display.println(now.minute(), DEC);
  } else {
    display.setCursor(21, line2);
    display.println(now.minute(), DEC);
  }
  currentMin=now.minute();
  display.setCursor(34, line2);
  display.println(":");

  /***** Add leading 0 To Seconds & display Seconds *****/
  if (now.second() < 10){
    display.setCursor(41, line2);
    display.print("0");
    display.println(now.second(), DEC);
  } else {
    display.setCursor(41, line2);
    display.println(now.second(), DEC);   
  }

  /***** Display AM-PM *****/
  display.setCursor(56, line2);
  display.println(ampm); 

  /***** Display Temp *****/
  display.setCursor(73, line2);
  display.print("TempF: " );
  //display.setCursor(70, 10);
  display.println(tempF, 0);

  /***** Display Gate Count *****/
  display.setTextSize(1);
  display.setCursor(0, line3);
  display.print("Exiting: ");
  display.setTextSize(2); 
  
  display.setCursor(50, line3);           
  display.println(totalDailyCars);
  display.setTextSize(1);
  display.setCursor(0, line5);
  display.print("In Park: ");
  display.setTextSize(2); 
  display.setCursor(50, line5);
  display.println(carCounterCars-totalDailyCars);

  display.display();

  //Save Hourly Totals
  if (now.minute()==0 && now.second()==0)
  {
    HourlyTotals();
  }


  /* 24/10/14 - Both beams are normally open. Optocoupler reads HIGH when sensors are NOT tripped
  changed code to read inverse of pin. Changing pinmode from pullup or pulldown made no difference 
  Continually Read state of sensors */
  magSensorState=!digitalRead(magSensorPin);
  beamSensorState=!digitalRead(beamSensorPin);


  /***** DETECTING CARS *****/      
  /* Sense Vehicle & Count Cars Exiting
  Both sensors HIGH when vehicle sensed, Normally both normally open (LOW)
  Both Sensors need to be active to start sensing vehicle Magnotometer senses vehicle not people
  Then Beam confirms vehicle is present and then counts car after vehicle passes
  IMPORTANT: Magnotometer will bounce as a single vehicle passes. */
  if (magSensorState != lastmagSensorState && magSensorState == 1) // if 2nd beam switches to High set Timer
  {
    magSensorTripTime = millis(); // double check. may need a way to reset if there is a bounce 11/3/24
  }
  if (beamSensorState != lastbeamSensorState && beamSensorState == 1) // if 2nd beam switches to High set Timer
  {
    beamSensorTripTime = millis(); // double check. may need a way to reset if there is a bounce 11/3/24
  }

  if ((magSensorState == 1) && (beamSensorState == 1)) 
  {
//    if (millis()-beamSensorTripTime >= carDetectMillis) // if beamsensor is blocked for x millis
//    {
    carPresentFlag = 1; // when both detectors are high, set flag car is in detection zone. Then only watch Beam Sensor
    carDetectedMillis = beamSensorTripTime; // Freeze time when car entered detection zone (use to calculate TimeToPass in millis
//    }
    // DEBUG CODE
    
    DateTime now = rtc.now();
    char buf3[] = "YYYY-MM-DD hh:mm:ss"; //time of day when detector was tripped
    Serial.print("Detector Triggered = ");
    Serial.print(now.toString(buf3));
    Serial.print(", beamSensorState = ");
    Serial.print(beamSensorState);
    Serial.print(", TimeToPass = ");
    Serial.print(millis() - carDetectedMillis);
    Serial.print(", Car Number Being Counted = ");         
    Serial.println (totalDailyCars+1) ;  //add 1 to total daily cars so car being detected is synced
    
    mqtt_client.publish(MQTT_PUB_TOPIC12, String(beamSensorState).c_str());  // publishes beamSensor State goes HIGH

    /* When both Sensors are tripped, car is in the detection zone. carPresentFlag=1
    Note: magSensor will trip multiple times while car is in detection zone
    when car clears detection zone & beam sensor remains LOW for period of time
    Then Reset Car Present Flag to 0 */
    while (carPresentFlag == 1)
    {
      beamSensorState = !digitalRead(beamSensorPin); // BSS-Beam Sensor is now priority. Ignore magSensor until car clears detection zone
      TimeToPassMillis=millis()-carDetectedMillis; //   TTPm-While car in detection zone, Record time while car is passing         
    
      if (beamSensorState != lastbeamSensorState && beamSensorState == 0) // if beam bounces set timer
      {
        beamSensorBounceTime = millis(); // double check. may need a way to reset if there is a bounce 11/3/24
      }
      /* If beamSensor is LOW CHECK CAR HAS CLEARED AND BREAK LOOP ################
      force count & reset if there is an undetectable car present 12/25/23
      This section may be removed with new beam sensor 10/13/24                 
      Check added 12/21/23 to ensure no car is present for x millis
      this section will determine if beam sensor is low not caused by a bounce */
      if (beamSensorState == 0)
      {
       // if ((millis() - beamSensorBounceTime) > 500)  //Allow for 500 millis bounce
       // {
          TimeToPassMillis=millis()-carDetectedMillis; //   TTPm-While car in detection zone, Record time while car is passing 
          carPresentFlag = 0;  //Reset carPresentFlag 
          updateCarCount(); // update Daily Totals and write data to file
      //  }
      }  // end of car passed check
    } // end of Car in detection zone (while loop)
  } /* End if when both Beam Sensors are HIGH */
  /***** END OF CAR DETECTION *****/

    


  //Added to kepp mqtt connection alive 10/11/24 gal
  if  ((currentMillis - start_MqttMillis)> (mqttKeepAlive*1000))
  {
      KeepMqttAlive();
  }

  lastbeamSensorState = beamSensorState;
  lastmagSensorState = magSensorState;
} /***** Repeat Loop *****/