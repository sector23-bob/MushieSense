#include <Adafruit_RGBLCDShield.h>
#include "Adafruit_SHT31.h"
#include <utility/Adafruit_MCP23017.h>

#include <Arduino.h>
#include <RTClib.h>
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
#define LOG_CNT 10 // Number of loops to average values over before logging
#define LCD_MULT 5 // Number of logging events before LCD update

/*
 * The SHT31's heater should be toggled about every 30 seconds to ensure proper functioning
 */
#define HEATER_MULT 25

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

// Real-time clock
RTC_PCF8523 rtc;

// Loop counter
uint16_t loopCnt = 0;

// Arrays for sensor readings
float tmpVals[LOG_CNT];
float humVals[LOG_CNT];
float co2Vals[LOG_CNT];

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
  char buff[1024];
  
  sprintf(buff, "Temp: %f F", t);
  lcd.setCursor(0,0);
  lcd.print(buff);

  sprintf(buff, "Hum: %f pct", h);
  lcd.setCursor(0,1);
  lcd.print(buff);
}

/*!
  @brief Do something if calculated values exceed defined parameters TODO
  @param tmpAvg Average temperature calculated
  @param humAvg Average humidity calculated
  @param co2Avg Average CO2 calculated TODO
 */
void freakOut(float t, float h, float c) {
    bool tmpExceed = ((t > MAXTEMP) or (t < MINTEMP));
    bool humExceed = ((h > MAXHUM) or (h < MINHUM));
    bool co2Exceed = false; //((c > MAXCO2) or (c < MINCO2)); // TODO
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
  @param c eCO2 calculated TODO
  @param now Time of logging event
 */
void logData(float t, float h, float c, DateTime now) {
  char buff[1024];
  sprintf(buff, "%s | %f | %f | %f ", now.timestamp().c_str(), t, h, c);
  Serial.println(buff);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Initializing...");
  int time = millis();

  /*** Initialize sensor arrays ***/
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
  rtc.start();
  int offset = 0; // Just in case
  rtc.calibrate(PCF8523_TwoHours, offset);
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
  if (sht31.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");
  // End SHT31
    
  /*** End sensors test ***/
  
  time = millis() - time; // TODO - log this?
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");

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
    float tmpAvg = (tmpSum/LOG_CNT);
    float humAvg = humSum/LOG_CNT;
    float co2Avg = co2Sum/LOG_CNT;
    logData(tmpAvg, humAvg, co2Avg, rtc.now());

    if (loopCnt % LCD_MULT*LOG_CNT == 0) {
      lcd.clear();
      updateDisplay(tmpAvg, humAvg, co2Avg);
    }

    // Reset sensor arrays
    memset(tmpVals, 0.0, sizeof(tmpVals));
    memset(humVals, 0.0, sizeof(humVals));
    memset(co2Vals, 0.0, sizeof(co2Vals));
  
    if (loopCnt % HEATER_MULT*LOG_CNT == 0) {
      // Do that maintenance
      enableHeater = !enableHeater;
      sht31.heater(enableHeater);
      Serial.print("Heater Enabled State: ");
      if (sht31.isHeaterEnabled()) {
        Serial.println("ENABLED");
      }
      else {
        Serial.println("DISABLED");
      }

      // Reset loop
      loopCnt = 0;
    }
  }
  /*** End housekeeping ***/
    
  delay(DELAY);

}
