#include "Adafruit_SHT31.h"

#include <Arduino.h>
#include <FS.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

// Colors
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

// Not Colors
// Control values
#define LOG_CNT 30    // Number of loops to average values over before logging
#define DELAY   120   // Delay per loop, in ms
#define HEATER_DELAY  TimeSpan(0, 0, 0, 30)   // Delay between heater cycle
#define WRITE_DELAY   TimeSpan(0, 0, 5, 0)    // Time between disk writes
#define FAN_DELAY     TimeSpan(0, 0, 15, 0)   // Time between fan activation
#define FAN_ON_TIME   TimeSpan(0, 0, 5, 0)    // How long the fan is on for

// Climate control limits
#define MAXTEMP 90.0    // Set ref. Stamets
#define MINTEMP 55.0    // Set ref. Stamets
#define MAXHUM  90.0    // Set ref. Stamets
#define MINHUM  25.0    // Set ref. Stamets
#define MAXCO2  0.0     // TODO - set this ref. Stamets
#define MINCO2  0.0     // TODO - set this ref. Stamets

// RFM9x module stuff TODO
#define RFM95_CS    2     // "E" pin for RFM9x
#define RFM95_RST   16    // "D" pin for RFM9x
#define RFM95_INT   15    // "B" pin for RFM9x
#define RFM95_FREQ  915.0 // Frequency for radio
#define MAX_MSG     252   // Maximum message size in bytes     

// Pin for fan control TODO
#define FAN_PIN   2   // We'll control the fan with GPIO2

// Temp/humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Real-time clock
RTC_PCF8523 rtc;

// Enable/disable heater on SHT31
bool enableHeater = false;

// Enable/disable fan on GIO2
bool enableFan = false;

// Alarms
DateTime fanOnAlarm;    // Turn on fan
DateTime fanOffAlarm;   // Turn off fan
DateTime heaterAlarm;   // Toggle heater on/off
DateTime logAlarm;      // Writing to SD card

// SD card pin
const int sdcardPin = 15;

// Loop counter
uint8_t loopCnt = 0;

// Log string
String logString;

// Arrays for sensor readings
float tmpVals[LOG_CNT];
float humVals[LOG_CNT];
float co2Vals[LOG_CNT];

/*!
  @brief Do something if calculated values exceed defined parameters TODO
  @param tmpAvg Average temperature calculated
  @param humAvg Average humidity calculated
  @param co2Avg Average CO2 calculated TODO
 */
void freakOut(float t, float h, float c) {
    bool tmpExceed = (t > MAXTEMP) or (t < MINTEMP);
    bool humExceed = (h > MAXHUM) or (h < MINHUM);
    bool co2Exceed = false; //(c > MAXCO2) or (c < MINCO2); // TODO
    // Do something?
}

/*!
  @brief Functionality to handle sensor errors TODO
 */
void handleSensorError() {
  ;
}

/*!
  @brief Perform logging actions for recorded values
  @param loopTime Timestamp from the loop calling this log event
 */
void doLogging(DateTime loopTime) {
  // Get averages
  float tmpSum = 0.0;
  float humSum = 0.0;
  float co2Sum = 0.0;

  for (int i = 0; i < LOG_CNT; i++) {
    tmpSum += tmpVals[i];
    humSum += humVals[i];
    co2Sum += co2Vals[i];
  }
  float tmpAvg = tmpSum/LOG_CNT;
  float humAvg = humSum/LOG_CNT;
  float co2Avg = co2Sum/LOG_CNT;

  // Add data to log string
  char message[1024];
  sprintf(message, "%s | %f | %f | %f ", loopTime.timestamp().c_str(), tmpAvg, humAvg, co2Avg);
  logString += String(message) + "\n";

  // Send LoRa message via RFM9x TODO
}

/*!
  @brief Write to a log file in memory using LittleFS
  @param message Message to be written
  @param loopTime Timestamp from the loop calling this log event
 */
void writeToLog(const char * message, DateTime loopTime) {
  char buf[] = "YYYYMMDD-hh";
  char * date = loopTime.toString(buf);
  char fname[64];
  sprintf(fname, "/%s.log", date);

  if (! SD.exists(fname)) {
    File newFile = SD.open(fname, FILE_WRITE);
    if (newFile) {
      Serial.print("Opened new log file: "); Serial.println(fname);
      newFile.print(loopTime.timestamp()); newFile.println(" | Begin log");
      newFile.close();
    } else {
      // Throw some kind of error?
      Serial.print("Couldn't open log file: "); Serial.println(fname);
    }
  } else {
    Serial.print("Logging to file: "); Serial.println(fname);
  }

  File logFile = SD.open(fname, FILE_WRITE);
  if (logFile) {
    logFile.print(message);
    logFile.close();
  } else {
    Serial.print("Couldn't log to file: "); Serial.println(fname);
  }
}

void setup() {
  Serial.begin(115200);
  while (! Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Initializing...");
  int time = millis();

  /*** Set up RTC ***/
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  //if (! rtc.initialized() || rtc.lostPower()) {
  if (true) {
    Serial.println("RTC is not initialized, setting...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }  

  rtc.start();
  //int offset = 0; // There's a whole thing about this, I don't know that it's super important
  //rtc.calibrate(PCF8523_TwoHours, offset);
  Serial.printf("RTC up at %s\n", rtc.now().timestamp().c_str());
  /*** End RTC setup ***/

  /*** Record start time for setup ***/
  DateTime startTime = rtc.now();

  /*** Initialize variables ***/
  memset(tmpVals, 0.0, sizeof(tmpVals));
  memset(humVals, 0.0, sizeof(humVals));
  memset(co2Vals, 0.0, sizeof(co2Vals));
  logString = "";
  
  /*** Set up SD card ***/
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(sdcardPin)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println(" card initialized.");
  /*** End SD card ***/

  /*** Test sensors ***/
  // SHT31
  Serial.println("SHT31 test");
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }

  Serial.print("Heater Enabled State: ");
  if (sht31.isHeaterEnabled()) {
    Serial.println("ENABLED");
  }
  else {
    Serial.println("DISABLED");
  }
  heaterAlarm = startTime + HEATER_DELAY;
  // End SHT31
  /*** End sensors test ***/

  /*** Fan ***/
  pinMode(FAN_PIN, OUTPUT);
  // Manual reset
  digitalWrite(FAN_PIN, LOW);
  delay(10);
  digitalWrite(FAN_PIN, HIGH);
  delay(10);
  digitalWrite(FAN_PIN, LOW);
  fanOnAlarm = startTime + FAN_DELAY;
  fanOffAlarm = startTime;
  
  /*** Set up LoRa 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
   End LoRa setup ***/
  
  logAlarm = startTime + WRITE_DELAY;

  time = millis() - time; // TODO - log this?
  Serial.printf("Setup took %d ms\n", time);
  Serial.flush();
}


void loop() {
  /*** Record start time for this loop ***/
  DateTime loopTime = rtc.now();
  
  /*** Read sensor vals ***/
  float t = sht31.readTemperature()*1.8 + 32.0; // Convert to F
  float h = sht31.readHumidity();
  float c = 0.0; // CO2 sensor TODO
  
  if (! isnan(t)) {
    tmpVals[loopCnt % LOG_CNT] = t;
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {
    humVals[loopCnt % LOG_CNT] = h;
  } else { 
    Serial.println("Failed to read humidity");
  }

  if (! isnan(c)) {
    co2Vals[loopCnt % LOG_CNT] = c;
  } else {
    Serial.println("Failed to read CO2");
  }
  /*** End sensor read ***/

  // Do we need to freak out?
  freakOut(t, h, c);

  /*** Do some housekeeping - maintenance, logging, and output - once the loop count thresholds are reached ***/
  /*** Maintenance ***/
  // Toggle heater
  if (loopTime > heaterAlarm) {
    enableHeater = ! enableHeater;
    sht31.heater(enableHeater);
    heaterAlarm = heaterAlarm + HEATER_DELAY;
  }
  
  // Fan
  if (! enableFan and (loopTime > fanOnAlarm)) {
    Serial.println("Enabling fan");
    digitalWrite(FAN_PIN, HIGH);
    enableFan = true;
    fanOffAlarm = fanOnAlarm + FAN_ON_TIME;
    fanOnAlarm = fanOnAlarm + FAN_DELAY;
  }

  if (enableFan and (loopTime > fanOffAlarm)) {
    Serial.println("Disabling fan");
    digitalWrite(FAN_PIN, LOW);
    enableFan = false;
  }
  /*** End maintenance ***/

  /*** Logging ***/
  if (loopTime > logAlarm) {
    writeToLog(logString.c_str(), loopTime);
    logAlarm = logAlarm + WRITE_DELAY;
    logString = "";
  }
  
  if (++loopCnt % LOG_CNT == 0) {
    doLogging(loopTime);

    if (loopCnt > 200) {
      loopCnt = 0;
      Serial.printf("%s | %f | %f | %f\n", loopTime.timestamp().c_str(), t, h, c);
    }

    // Re-zero sensor read arrays
    memset(tmpVals, 0.0, sizeof(tmpVals));
    memset(humVals, 0.0, sizeof(humVals));
    memset(co2Vals, 0.0, sizeof(co2Vals));
  }
  /*** End logging ***/
  /*** End housekeeping ***/
  
  delay(DELAY);

}
