/*
Gate Counter by Greg Liebig gliebig@sheboyganlights.org
Initial Build 12/5/2023 12:15 pm

Changelog
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

// ******************** VARIBLES *******************
#define magSensorPin 32 // Pin for Magnotometer Sensor
#define beamSensorPin 33  //Pin for Reflective Scensor
#define PIN_SPI_CS 5 // SD Card CS GPIO5
// #define MQTT_KEEPALIVE 30 //removed 10/16/24
#define FWVersion "24.10.23.2" // Firmware Version
#define OTA_Title "Gate Counter" // OTA Title
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
int line1 =0;
int line2 =9;
int line3 = 19;
int line4 = 30;
int line5 = 42;
int line6 = 50;
int line7 = 53;

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

// MQTT TOPIC DEFINITIONS
#define THIS_MQTT_CLIENT "espGateCounter" // Look at line 90 and set variable for WiFi Client secure & PubSubCLient 12/23/23
int mqttKeepAlive = 30; // publish temp every x seconds to keep MQTT client connected
// Publishing Topics 
#define MQTT_PUB_TOPIC0 "msb/traffic/exit/hello"
#define MQTT_PUB_TOPIC1 "msb/traffic/exit/temp"
#define MQTT_PUB_TOPIC2 "msb/traffic/exit/time"
#define MQTT_PUB_TOPIC3 "msb/traffic/exit/count"
#define MQTT_PUB_TOPIC4 "msb/traffic/exit/inParkCars"
#define MQTT_PUB_TOPIC5 "msb/traffic/enter/hour1"
#define MQTT_PUB_TOPIC6 "msb/traffic/enter/hour2"
#define MQTT_PUB_TOPIC7 "msb/traffic/enter/hour3"
#define MQTT_PUB_TOPIC8 "msb/traffic/enter/hour4"
#define MQTT_PUB_TOPIC9 "msb/traffic/enter/DayTot"
#define MQTT_PUB_TOPIC10 "msb/traffic/enter/ShoTot"
#define MQTT_PUB_TOPIC11 "msb/traffic/exit/debug/timeout"
#define MQTT_PUB_TOPIC12 "msb/traffic/exit/debug/beamSensorState"

// Subscribing Topics (to reset values)
#define MQTT_SUB_TOPIC0  "msb/traffic/enter/DayTot"
#define MQTT_SUB_TOPIC1  "msb/traffic/exit/resetcount"


//const uint32_t connectTimeoutMs = 10000;
uint16_t connectTimeOutPerAP=5000;
const char* ampm ="AM";
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -21600;
const int   daylightOffset_sec = 3600;
int16_t tempF;

char buf2[25] = "YYYY-MM-DD hh:mm:ss";

// ******** RESET COUNTS ON REBOOT *******
unsigned int currentDay;
unsigned int currentHour;
unsigned int currentMin;
unsigned int currentSec;
unsigned int lastCalDay;
int totalDailyCars;
int totalShowCars;
unsigned int daysRunning;
int inParkCars; // cars in park
int carCounterCars =0;
int carsHr1 =0; // total cars hour 1
int carsHr2 =0; // total cars hour 2
int carsHr3 =0; // total cars hour 3
int carsHr4 =0; // total cars hour 4
int sensorBounceCount=0;
int sensorBounceRemainder;
bool sensorBounceFlag;

int carPresentFlag = 0;  // used to flag when car is in the detection zone

int NoCarFlag = 0;  // used to clear car in detection zone. May not be necessary
unsigned long TimeToPassMillis; // time car is in detection Zone
unsigned long MinDetectionTime = 3000; // Minimum time to pass through beamSensor detection zone
unsigned long lastTimeToPassMillis = 0;
unsigned long beamSensorLowMillis = 0; // the time when beam sensor changes low 
unsigned long lastbeamSensorLowMillis = 0; // last time beam sensor was low for bounce checking when car is in detection zome and beam sensor state changes
unsigned long beamSensorHighMillis;
unsigned long bounceTimerMillis;


/********** Bounce Times **********/
unsigned long ignoreBounceTimeMillis = 1000;  // Maximum time to ingnore a beam state change while car in detection zone
unsigned long nocarTimeoutMillis = 900; // Time required for beamSensor to stay low to clear car in detection zone
unsigned long nocarTimerMillis = 0; // Time delay to allow car to pass before checking for HIGN pin

//unsigned long highMillis = 0; //Grab the time when the vehicle sensor is high
unsigned long carDetectedMillis;  // Grab the ime when sensor 1st trips
unsigned long lastcarDetectedMillis;  // Grab the ime when sensor 1st trips
unsigned long wifi_connectioncheckMillis = 5000; // check for connection every 5 sec
unsigned long mqtt_connectionCheckMillis = 20000; // check for connection
unsigned long start_MqttMillis; // for Keep Alive Timer
unsigned long start_WiFiMillis; // for keep Alive Timer
unsigned long currentMillis; // loop timer

File myFile; //used to write files to SD Card
File myFile2;

// **********FILE NAMES FOR SD CARD *********
const String fileName1 = "/DailyTot.txt"; // /Gate/DailyTot.txt file to store daily counts in the event of a Failure
const String fileName2 = "/ShowTot.txt";  // /Gate/ShowTot.txt file to store season total counts
const String fileName3 = "/CalDay.txt"; // /Gate/CalDay.txt file to store current day number
const String fileName4 = "/RunDays.txt"; // /Gate/RunDays.txt file to store days since open
const String fileName5 = "/DailySummary.csv"; // /Gate/DailySummary.csv Stores Daily Totals by Hour and total
const String fileName6 = "/GateCount.csv"; // /Gate/GateCount.csv file to store all car counts for season (was MASTER.CSV)
const String fileName7 = "/SensorBounces.csv"; // /Gate/SensorBounces.csv file to store all car counts for season (was MASTER.CSV)



char days[7][4] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char months[12][4] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

// var ************* DISPLAY SIZE ************
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire, -1);


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
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200,"text/plain","ok");
    delay(2000);
    ESP.restart();
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
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  payload[length] = '\0';
 
  if (strcmp(topic, MQTT_SUB_TOPIC0) == 0) {
     carCounterCars = atoi((char *)payload);
//     Serial.println(" Car Counter Updated");
    }
  
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
  }
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
void getInitialDailyTotal()   // open DAILYTOT.txt to get initial dailyTotal value
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
    Serial.print(F("SD Card: Cannot open the file DailyTotal.TXT"));
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
      Serial.print(F("SD Card: Cannot open the file TOTAL.TXT"));
    }
}

void getInitialDayRunning()   // Days the show has been running)
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
       Serial.print(F("SD Card: Cannot open the file Running.TXT"));
    }
} 

void getLastCalDay()  // get the last calendar day used for reset daily counts)
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
      Serial.print(F("SD Card: Cannot open the file Running.TXT"));
    }
} 

/***** UPDATE TOTALS TO SD CARD *****/
void HourlyTotals()
{
  if (currentHour == 19)
  {
    carsHr1 = totalDailyCars;
  }
  if (currentHour == 20)
  {
    carsHr2 = totalDailyCars-carsHr1;
  }
  if (currentHour == 21)
  {
    carsHr3 = totalDailyCars-(carsHr1+carsHr2);
  }
  if (currentHour == 22)
  {
    carsHr4 = totalDailyCars;
  }
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

void WriteTotals()
{
  DateTime now = rtc.now();
  tempF = ((rtc.getTemperature()*9/5)+32);
  char buf2[] = "YYYY-MM-DD hh:mm:ss";
  Serial.print(now.toString(buf2));
  Serial.print(", Temp = ");
  Serial.print(tempF);
  Serial.print(", ");
//  totalDailyCars ++;     
//  totalDailyCars;     
  Serial.print(totalDailyCars) ;  
  // open file for writing Car Data
  myFile2 = SD.open(fileName5, FILE_APPEND);
  if (myFile) 
  {
    myFile2.print(now.toString(buf2));
    myFile2.print(", ");
    myFile2.print (tempF); 
    myFile2.print(", "); 
    myFile2.print (carsHr1) ; 
    myFile2.print(", ");
    myFile2.println(carsHr2);
    myFile2.print(", ");
    myFile2.println(carsHr3);
    myFile2.print(", ");
    myFile2.println(carsHr4);
    myFile2.print(", ");
    myFile2.println(totalDailyCars);
    myFile2.close();
    Serial.println(F(" = Daily Summary Recorded SD Card."));
    // Publish Totals
    mqtt_client.publish(MQTT_PUB_TOPIC1, String(tempF).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC2, now.toString(buf2));
    mqtt_client.publish(MQTT_PUB_TOPIC3, String(totalDailyCars).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC4, String(inParkCars).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC5, String(carsHr1).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC6, String(carsHr2).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC7, String(carsHr3).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC8, String(carsHr4).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC9, String(totalDailyCars).c_str());
    mqtt_client.publish(MQTT_PUB_TOPIC10, String(totalShowCars).c_str());
  }
  else
  {
    Serial.print(F("SD Card: Issue encountered while attempting to open the file CarCount.csv"));
  }
}
/***** END OF FILE OPS *****/



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
 
  //***** Check AND/OR Prep Files for use ******/ 
  if (!SD.exists(fileName1))
  {
    Serial.println(F("DailyTot.txt doesn't exist. Creating file..."));
    // create a new file by opening a new file and immediately close it
    myFile2 = SD.open(fileName1, FILE_WRITE);
    myFile2.close();
    // recheck if file is created or not & write Header
   }

   if (!SD.exists(fileName2))
   {
     Serial.println(F("ShowTot.txt doesn't exist. Creating file..."));
     // create a new file by opening a new file and immediately close it
     myFile2 = SD.open(fileName2, FILE_WRITE);
     myFile2.close();
     // recheck if file is created or not & write Header
   }

   if (!SD.exists(fileName3))
   {
     Serial.println(F("CalDay.txt doesn't exist. Creating file..."));
     // create a new file by opening a new file and immediately close it
     myFile2 = SD.open(fileName3, FILE_WRITE);
     myFile2.close();
          // recheck if file is created or not & write Header
   }

  if (!SD.exists(fileName4))
  {
    Serial.println(F("RunDays.txt doesn't exist. Creating file..."));
    // create a new file by opening a new file and immediately close it
    myFile2 = SD.open(fileName4, FILE_WRITE);
    myFile2.close();
    // recheck if file is created or not & write Header
  }
  
  if (!SD.exists(fileName5))
  {
    Serial.println(F("DailySummary.csv doesn't exist. Creating file..."));
    // create a new file by opening a new file and immediately close it
    myFile2 = SD.open(fileName5, FILE_WRITE);
    myFile2.close();
    // recheck if file is created or not & write Header
  }
  // recheck if file is created or not & write Header
  if (SD.exists(fileName6))
  {
     Serial.println(F("GateCount.csv exists on SD Card."));
     myFile = SD.open(fileName6, FILE_APPEND);
     myFile.println("Date Time,Pass Timer,NoCar Timer,Bounces,Car#,Cars In Park,Temp,Last Car Millis, This Car Millis,Bounce Flag,Millis");
     myFile.close();
     Serial.println(F("Header Written to GateCount.csv"));
  } 
  else
  {
    Serial.println(F("GateCount.csv doesn't exist on SD Card."));
  }
  
  if (!SD.exists(fileName7))
  {
    Serial.println(F("SensorBounces.csv doesn't exist. Creating SensorBounces.csv file..."));
    // create a new file by opening a new file and immediately close it
    myFile2 = SD.open(fileName7, FILE_WRITE);
    myFile2.close();
  }
  // recheck if file is created or not & write Header
  if (SD.exists(fileName7))
  {
    Serial.println(F("SensorBounces.csv exists on SD Card."));
    myFile2 = SD.open(fileName7, FILE_APPEND);
    //("DateTime\t\t\tPassing Time\tLast High\tDiff\tLow Millis\tLast Low\tDiff\tBounce #\tCurent State\tCar#" )
    myFile2.println("DateTime,TimeToPass,Last Beam Low,Beam Low,Diff,Beam State, Bounce#,Car#");
    myFile2.close();
    Serial.println(F("Header Written to SensorBounces.csv"));
  }
  else
  {
    Serial.println(F("SensorBounces.csv doesn't exist on SD Card."));
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

getInitialDailyTotal();
getInitialDayRunning();
getLastCalDay();
getShowTotal();


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
   DateTime now = rtc.now();
   tempF=((rtc.getTemperature()*9/5)+32);
   currentMillis = millis(); 

  /*Reset Gate Counter at 5:10:00 pm before guests arrive at Igloo and allow any remaining helpers
  not to affect show counts*/
  if ((now.hour() == 17) && (now.minute() == 10) && (now.second() == 0))
  {
     totalDailyCars = 0;
     updateDailyTotal();
  }    
  
  //Write Totals at 9:20:00 pm. Gate should close at 9 PM. Allow for any cars in line get through
  if ((now.hour() == 21) && (now.minute() == 20) && (now.second() == 0))
  {
    WriteTotals();
  }
  
  // Reset/Update Counts wwhen Day Changes
  if (now.day() != lastCalDay)
  {
    currentDay=now.day();
    updateCalDay();
    totalDailyCars =0;
    updateDailyTotal();
    daysRunning++;
    updateDaysRunning();

  }
   

  // non-blocking WiFi and MQTT Connectivity Checks
  if (wifiMulti.run() == WL_CONNECTED)
  {
     // Check for MQTT connection only if wifi is connected
     if (!mqtt_client.connected())
     {
       if (currentMillis - start_MqttMillis > mqtt_connectionCheckMillis)
       {
          Serial.print("hour = ");
          Serial.println(currentHour);
          Serial.println("Attempting MQTT Connection");
          MQTTreconnect();
          start_MqttMillis = currentMillis;
      }   
    } 
    else
    {
         //keep MQTT client connected when WiFi is connected
         mqtt_client.loop();
    }
    } 
    else
    {
       // Reconnect WiFi if lost, non blocking
       if ((currentMillis - start_WiFiMillis) > wifi_connectioncheckMillis)
       {
          setup_wifi();
          start_WiFiMillis = currentMillis;
       }
    }    

   //Required for OTA Programming
   ElegantOTA.loop();

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
      
      // Convert 24 hour clock to 12 hours
      currentHour = now.hour();
      if (currentHour <12)
      {
               ampm ="AM";
      }
      else
      {
               ampm ="PM";
      }
      if (currentHour > 12 )
      {
        currentHour = now.hour() - 12;
      }
      else
      {
        currentHour = now.hour();
      }

      //Display Time
      //add leading 0 to Hours & display Hours
      display.setTextSize(1);

      if (currentHour < 10){
        display.setCursor(0, line2);
        display.print("0");
        display.println(currentHour, DEC);
      }else{
        display.setCursor(0, line2);
        display.println(currentHour, DEC);
      }
        display.setCursor(14, line2);
      display.println(":");
 
      //Add leading 0 To Mintes & display Minutes 
      //  display.setTextSize(1);
      if (now.minute() < 10)
      {  
        display.setCursor(20, line2);
        display.print("0");
        display.println(now.minute(), DEC);
      }
      else
      {
        display.setCursor(21, line2);
        display.println(now.minute(), DEC);
      }
      currentMin=now.minute();
      display.setCursor(34, line2);
      display.println(":");

      //Add leading 0 To Seconds & display Seconds
      //  display.setTextSize(1);
      if (now.second() < 10){
        display.setCursor(41, line2);
        display.print("0");
        display.println(now.second(), DEC);
      }else{
        display.setCursor(41, line2);
        display.println(now.second(), DEC);   
      }

      // Display AM-PM
      display.setCursor(56, line2);
      display.println(ampm); 

      // Display Temp
      // display.setTextSize(1);
      display.setCursor(73, line2);
      display.print("TempF: " );
      //display.setCursor(70, 10);
      display.println(tempF, 0);

      // Display Gate Count
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

      // 24/10/14 - Both beams are normally open. Optocoupler reads HIGH when sensors are NOT tripped
      // changed code to read inverse of pin. Changing pinmode from pullup or pulldown made no difference
      int magSensorState=!digitalRead(magSensorPin);
      int beamSensorState=!digitalRead(beamSensorPin);
      int LastbeamSensorState = 0; // added 2024/10/15 



// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@     
      // Sense Vehicle & Count Cars Exiting
      // Both sensors HIGH when vehicle sensed, Normally both normally open (LOW)
      // Both Sensors need to be active to start sensing vehicle Magnotometer senses vehicle not people
      // Then Beam confirms vehicle is present and then counts car after vehicle passes
      // IMPORTANT: Magnotometer will bounce as a single vehicle passes. 
      // ignore if beam sensor bounces for x millis 
         //Serial.print("detected mag = ");
         //Serial.print(magSensorState);
         //Serial.print("\tbeam = ");
         //Serial.println(beamSensorState);
      if ((magSensorState == 1) && (beamSensorState == 1)) {

          sensorBounceCount = 0; //Reset bounce counter
          carPresentFlag = 1; // when both detectors are high, set flag car is in detection zone. Then only watch Beam Sensor
          carDetectedMillis = millis(); // Freeze time when car entered detection zone (use to calculate TimeToPass in millis)
          beamSensorLowMillis = millis();  // This equals carDetectedMillis before any bouncing and will reset if a bounce happens
          bounceTimerMillis = 0; // Reset bounceTimer to 0 for bounce check
//          beamSensorLowMillis = millis()-carDetectedMillis;
//          lastSensorState=HIGH;

// Used for debugging
          DateTime now = rtc.now();
          char buf3[] = "YYYY-MM-DD hh:mm:ss"; //time of day when detector was tripped
          Serial.print("Detector Triggered = ");
          Serial.print(now.toString(buf3));
//          Serial.print(" \t ");
          Serial.print(", beamSensorState = ");
          Serial.print(beamSensorState);
          Serial.print(", TimeToPass = ");
          Serial.print(millis() - carDetectedMillis);
          Serial.print(", Car Number Being Counted = ");         
          Serial.println (totalDailyCars+1) ;  //add 1 to total daily cars so car being detected is synced
          mqtt_client.publish(MQTT_PUB_TOPIC6, String(beamSensorState).c_str());  // publishes beamSensor State


          // When both Sensors are tripped, car is in the detection zone.
          // magSensor will trip multiple times while car is in detection zone
          // figure out when car clears detection zone & beam sensor remains LOW for period of time if it bounces
          // Then Reset Car Present Flag to 0
          while (carPresentFlag == 1) {
            beamSensorState = !digitalRead(beamSensorPin); // BSS-Beam Sensor is now priority. Ignore magSensor until car clears detection zone
            TimeToPassMillis=millis()-carDetectedMillis; //   TTPm-While car in detection zone, Record time while car is passing         
/*            beamSensor may bounce while vehicle is in detection zone from high to HIGH to LOW to HIGH
              This loop is used to ignore those bounces
              if beam detector state changes from HIGH to LOW for more than ignoreBounceTimeMillis car cleared sensor & increment count
              If it remains HIGH car is in detection zone
              Added publishing state changes of beamSensor to MQTT 10/13/24
              If beamSensorState bounces from LOW to HIGH when car is in detection zone CarPresentFlag =1
 */
               if ((beamSensorState != LastbeamSensorState) && (beamSensorState == 1)) {
                lastbeamSensorLowMillis=beamSensorLowMillis;  // LBSLM-Save last time beamSensor was LOW
               // mqtt_client.publish(MQTT_PUB_TOPIC6, String(beamSensorState).c_str());
               }

/*            If beamSensorState bounces from HIGH to LOW when car is in detection zone (CarPresentFlag = 1)
              We need to determine if it's just a bounce or if the car left the detection zone
              by calculating the time of the bounce from high to low back to high
              if it happens quickly (less than 1 second) then ignore the bounce. If the beamState > a second then reset
              car present tag to 0 and count the car.
*/
              if ((beamSensorState != LastbeamSensorState)  && (beamSensorState == 0) && (NoCarFlag == 1)) {
                if ((sensorBounceCount == 0) ){
                  // Print header for debugging bounces
                  Serial.println("DateTime\t\tTTPm\tLBSLm\tBSLm\tDiff\t\tBSS\tBounce#\tCar#" ); 
                }
                sensorBounceCount ++;  // SBC-count the state changes from HIGH to LOW
                beamSensorLowMillis = millis()-carDetectedMillis;  // BSLm start timer when beamSensor Bounces LOW
                //Serial Print Bounce
                //Debugging Code Can be removed  **************************************************************************
                DateTime now = rtc.now();
                char buf2[] = "YYYY-MM-DD hh:mm:ss";
                Serial.print(now.toString(buf2));
                Serial.print("\t");
                Serial.print(TimeToPassMillis);
                Serial.print("\t");
                Serial.print(lastbeamSensorLowMillis); 
                Serial.print("\t");
                Serial.print(beamSensorLowMillis);
                Serial.print("\t");
                Serial.print(lastbeamSensorLowMillis-beamSensorLowMillis);
                Serial.print("\t");
                Serial.print(beamSensorState);                        
                Serial.print("\t");   
                Serial.print(sensorBounceCount);
                Serial.print("\t");
                Serial.print(totalDailyCars+1);
                Serial.println();
                         
                //Write Bounce info to file2 "DateTime\t\tTTPm\tLBSLm\tBSLm\tDiff\tBounce #\tBSS\tCar#" );  
                myFile2 = SD.open("/SensorBounces.csv", FILE_APPEND);
                if (myFile2) {
                  myFile2.print(now.toString(buf2));
                  myFile2.print(", "); 
                  myFile2.print(TimeToPassMillis);
                  myFile2.print(", "); 
                  myFile2.print (lastbeamSensorLowMillis) ; 
                  myFile2.print(", ");
                  myFile2.print(beamSensorLowMillis);
                  myFile2.print(", ");
                  myFile2.print(beamSensorLowMillis-lastbeamSensorLowMillis);
                  myFile2.print(", ");
                  myFile2.print(sensorBounceCount);
                  myFile2.print(" , ");
                  myFile2.print(beamSensorState);
                  myFile2.print(" , ");
                  myFile2.print(totalDailyCars+1); //Prints this Car millis
                  myFile2.println();
                  myFile2.close();
                  } else {
                    Serial.print(F("SD Card: Issue encountered while attempting to open the file GateCount.csv"));
                  }
                } // End IF when beamSensorState bounces LOW
/*
                If beamSensor is LOW CHECK CAR HAS CLEARED AND BREAK LOOP #################################################
                force count & reset if there is an undetectable car present 12/25/23
                This section may be removed with new beam sensor 10/13/24                 
                Check added 12/21/23 to ensure no car is present for x millis
                this section will determine if beam sensor is low not caused by a bounce
*/
                if (beamSensorState == 0)  {
                  // If no car is present and state does not change, then car has passed
                  if (((TimeToPassMillis) >= MinDetectionTime) ) { 
                    NoCarFlag = 0; //no car in detection zone
                  } 
 /*
                  //Resets if Loop sticks after 10 seconds and does not record a car.
                  if (millis() - carDetectedMillis > 10000) {
                    Serial.println("Timeout! No Car Counted");
                    mqtt_client.publish(MQTT_PUB_TOPIC5, String(totalDailyCars+1).c_str());
                    carPresentFlag=0;
                    break;
                  }
*/
                 } else {
                  NoCarFlag = 1;  // threshold not met to time out clearing car from detection zone
                 }
/*
             allow enough time for a car to pass and then make sure sensor remains low 10/13/24
             Conditions that must be met for a car to be clear and count the car ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
             Main Reset car passing timer
*/             
             // beamSensor is Low & no car Present OR if Bounce times out then record car not needed if beam sensor does not bounce 
//           if (((beamSensorState == LOW) && (NoCarFlag == 0)) || (beamSensorLowMillis-lastbeamSensorLowMillis > ignoreBounceTimeMillis) ) {
             if (((beamSensorState == 0) ) && (NoCarFlag == 0)) {
               Serial.print(now.toString(buf3));
               Serial.print(", Millis NoCarTimer = ");
               Serial.print(millis()-beamSensorLowMillis);
               Serial.print(", Time to pass = ");
               Serial.println(millis()-carDetectedMillis);
               //Serial.print(", ");
               //Serial.print(String("DateTime::TIMESTAMP_FULL:\t")+now.timestamp(DateTime::TIMESTAMP_FULL));
               //Serial.print(",1,"); 
               totalDailyCars ++;
               updateDailyTotal();
               totalShowCars ++;  
               updateShowTotal(); 
               // open file for writing Car Data
               //"Date Time,Pass Timer,NoCar Timer,TotalExitCars,CarsInPark,TempF"
               myFile = SD.open("/GateCount.csv", FILE_APPEND);
               if (myFile) {
                 myFile.print(now.toString(buf3));
                 myFile.print(", ");
                 myFile.print (millis()-carDetectedMillis) ; 
                 myFile.print(", ");
                 myFile.print (millis()-beamSensorLowMillis) ; 
                 myFile.print(", "); 
                 myFile.print (sensorBounceCount) ; 
                 myFile.print(", ");                       
                 myFile.print (totalDailyCars) ; 
                 myFile.print(", ");
                 myFile.print(carCounterCars-totalDailyCars);
                 myFile.print(", ");
                 myFile.print(tempF);
                 myFile.print(" , ");
                 myFile.print(lastcarDetectedMillis); //Prints car number being detected
                 myFile.print(" , ");
                 myFile.print(carDetectedMillis); //Prints car number being detected
                 myFile.print(", ");
                 myFile.print(sensorBounceFlag);
                 myFile.print(", ");
                 myFile.println(millis());
                 myFile.close();
                      
                 Serial.print(F("Car Saved to SD Card. Car Number = "));
                 Serial.print(totalDailyCars);
                 Serial.print(F(" Cars in Park = "));
                 Serial.println(carCounterCars-totalDailyCars);  
                 mqtt_client.publish(MQTT_PUB_TOPIC1, String(tempF).c_str());
                 mqtt_client.publish(MQTT_PUB_TOPIC2, now.toString(buf3));
                 mqtt_client.publish(MQTT_PUB_TOPIC3, String(totalDailyCars).c_str());
                 mqtt_client.publish(MQTT_PUB_TOPIC4, String(carCounterCars-totalDailyCars).c_str());
                 mqtt_client.publish(MQTT_PUB_TOPIC6, String(beamSensorState).c_str());
                 //snprintf (msg, MSG_BUFFER_SIZE, "Car #%ld,", totalDailyCars);
                 //Serial.print("Publish message: ");
                 //Serial.println(msg);
                 //mqtt_client.publish("msbGateCount", msg);
                 //}
               } else {
                 Serial.print(F("SD Card: Issue encountered while attempting to open the file GateCount.csv"));
               }
                  //Break while loop
                  carPresentFlag = 0;  //Reset Flag
                  sensorBounceFlag = 0;
                  TimeToPassMillis = 0;
                  lastcarDetectedMillis=carDetectedMillis;
              }  // end of car passed check

             LastbeamSensorState=beamSensorState;
             lastbeamSensorLowMillis=beamSensorLowMillis;
               
           } // end of Car in detection zone (while loop)

      } // Start looking for next time both beamSensor & magSesnor HIGH at same time
  //Added to kepp mqtt connection alive 10/11/24 gal
  if  ((currentMillis - start_MqttMillis)> (mqttKeepAlive*1000))
  {
      KeepMqttAlive();
  }
//loop forever looking for car and update time and counts
}