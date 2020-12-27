#include <Adafruit_RGBLCDShield.h>
#include "Adafruit_SHT31.h"
#include <utility/Adafruit_MCP23017.h>

#include <Arduino.h>
#include <RTClib.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>

// Colors
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

// Not Colors
#define LOG_CNT 20 // Number of loops to average values over before logging
#define LCD_MULT 2 // Number of logging events before LCD update
#define DELAY 120 // Delay per loop, in ms
#define MAXTEMP 90.0 // Set ref. Stamets
#define MINTEMP 55.0  // Set ref. Stamets
#define MAXHUM 90.0 // Set ref. Stamets
#define MINHUM 25.0 // Set ref. Stamets
#define MAXCO2 0.0 // TODO - set this ref. Stamets
#define MINCO2 0.0 // TODO - set this ref. Stamets

// LCD display
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// Temp/humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Enable/disable heater on SHT31
bool enableHeater = false;

// "Alarm" for heater on/off
DateTime heaterToggle;

// Real-time clock
RTC_PCF8523 rtc;

// Loop counter
uint8_t loopCnt = 0;

// Arrays for sensor readings
float tmpVals[LOG_CNT];
float humVals[LOG_CNT];
float co2Vals[LOG_CNT];

/*** Begin LittleFS helper functions ***/
/*! 
  @brief Write a message to a file. Stolen shamelessly from example code.
  @param path Path of file to be written
  @param message Message to write
 */
void writeFile(const char * path, const char * message) {
  File file = LittleFS.open(path, "w");
  if (! file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (! file.print(message)) {
    Serial.println("Write failed");
  }
  //delay(2000); // Make sure the CREATE and LASTWRITE times are different
  file.close();
}

/*!
  @brief Append a message to an existing file. Stolen shamelessly from example code.
  @param path Path of file to append to
  @param message Message to be appended
 */
void appendFile(const char * path, const char * message) {
  File file = LittleFS.open(path, "a");
  if (! file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (! file.print(message)) {
    Serial.println("Append failed");
  }
}
/*** End LittleFS helpers ***/

/*!
  @brief Read LCD buttons - slug code 
 */
void handleButtons() {
  uint8_t buttons = lcd.readButtons();
  
  if (buttons) {
    if (buttons & BUTTON_UP) {
      //lcd.setBacklight(RED);
    }
    if (buttons & BUTTON_DOWN) {
      //lcd.setBacklight(YELLOW);
    }
    if (buttons & BUTTON_LEFT) {
      //lcd.setBacklight(GREEN);
    }
    if (buttons & BUTTON_RIGHT) {
      //lcd.setBacklight(VIOLET);
    }
    if (buttons & BUTTON_SELECT) {
      //lcd.setBacklight(BLUE);
    }
  }
}

/*!
  @brief Updates serial and LCD displays, if present TODO
  @param tmpAvg Average temperature calculated
  @param humAvg Average humidity calculated
  @param co2Avg Average CO2 calculated TODO
 */
void updateDisplay(float t, float h, float c) {
  lcd.setCursor(0,0);
  lcd.printf("Temp: %f F", t);

  lcd.setCursor(0,1);
  lcd.printf("Hum: %f pct", h);
}

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
    if ((tmpExceed or humExceed) or co2Exceed) {
      lcd.setBacklight(RED);
    } else {
      lcd.setBacklight(GREEN);
    }
}

/*!
  @brief Functionality to handle sensor errors TODO
 */
void handleSensorError() {
  ;
}

/*!
  @brief Log data to serial and long-term storage TODO
  @param t Temperature
  @param h Humidity percent
  @param c eCO2 calculated @TODO
  @param now Time of logging event
 */
void logData(float t, float h, float c, DateTime now) {
  char message[1024];
  sprintf(message, "%s | %f | %f | %f ", now.timestamp().c_str(), t, h, c);

  //Serial.println(message);

  writeToLittleFSLog(message, now);
}
 
/*!
  @brief Write to a log file in memory using LittleFS
  @param message Message to be written
 */
void writeToLittleFSLog(const char * message, DateTime now) {
  char buf[] = "YYYYMMDD-hh";
  char * date = now.toString(buf);
  char fname[64];
  sprintf(fname, "/%s.log", date);
  if (! LittleFS.exists(fname)) {
    Serial.printf("Created file: %s\n", fname);
    writeFile(fname, (now.timestamp() + " | Begin log\n").c_str());
  }
  appendFile(fname, message);
  appendFile(fname, "\n");
  
  File file = LittleFS.open(fname, "r");
  Serial.printf("File size: %d\n", file.size());
}

void setup() {
  Serial.begin(115200);
  while (! Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Initializing...");
  int time = millis();

  /*** Initialize sensor read arrays ***/
  memset(tmpVals, 0.0, sizeof(tmpVals));
  memset(humVals, 0.0, sizeof(humVals));
  memset(co2Vals, 0.0, sizeof(co2Vals));

  /*** Initialize LCD ***/
  lcd.begin(16, 2);
  lcd.print("Gettin' ready!");

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
  //rtc.adjust(DateTime(2000, 1, 1, 8, 59, 30)); // Testing datetime
  rtc.start();
  //int offset = 0; // There's a whole thing about this, I don't know that it's super important
  //rtc.calibrate(PCF8523_TwoHours, offset);
  Serial.print("RTC up at "); Serial.println(rtc.now().timestamp());
  /*** End RTC setup ***/

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
  heaterToggle = rtc.now() + TimeSpan(0, 0, 0, 30);
  // End SHT31
    
  /*** End sensors test ***/

  /*** Set up LittleFS ***/
  Serial.println();
  Serial.println("Formatting LittleFS filesystem");
  LittleFS.format();
  Serial.println("Mount LittleFS");
  if (! LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }
  
  time = millis() - time; // TODO - log this?
  Serial.printf("Took %d ms\n", time);

  lcd.setBacklight(WHITE);
}


void loop() {
  handleButtons();
  
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
  // Maintenance
  // The heater should toggle every 30 seconds
  if (rtc.now() > heaterToggle) {
    enableHeater = ! enableHeater;
    sht31.heater(enableHeater);
    Serial.print("Heater Enabled State: ");
    if (sht31.isHeaterEnabled()) {
      Serial.println("ENABLED");
    }
    else {
      Serial.println("DISABLED");
    }
    heaterToggle = heaterToggle + TimeSpan(0, 0, 0, 30);
  }

  // Logging and output
  if (++loopCnt % LOG_CNT == 0) {
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

    // Log data
    logData(tmpAvg, humAvg, co2Avg, rtc.now());

    // Update display, reset loop if we're getting too big
    if (loopCnt % (LCD_MULT*LOG_CNT) == 0) {
      lcd.clear();
      updateDisplay(tmpAvg, humAvg, co2Avg);
      if (loopCnt > 200) {
        loopCnt = 0;
      }
    }

    // Re-zero sensor read arrays
    memset(tmpVals, 0.0, sizeof(tmpVals));
    memset(humVals, 0.0, sizeof(humVals));
    memset(co2Vals, 0.0, sizeof(co2Vals));
  }
  /*** End housekeeping ***/
  
  delay(DELAY);

}
