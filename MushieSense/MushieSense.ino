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
#define THRESHOLD 30 // Loop threshold for housekeeping. Keep this around 30 to keep the SHT31 happy.
#define MAXTEMP 90.0
#define MINTEMP 65.0
#define MAXHUM 90.0
#define MINHUM 75.0
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

uint8_t loopCnt = 0;

// Arrays for sensor readings
float tmpVals[THRESHOLD];
float humVals[THRESHOLD];
float co2Vals[THRESHOLD];

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
void updateDisplay(float tmpAvg, float humAvg, float co2Avg) {
    lcd.setCursor(0,0);
    lcd.print("Temp: "); lcd.print(tmpAvg); lcd.print(" F");
    Serial.print("Temp: "); Serial.print(tmpAvg); Serial.print("\t\t");
    
    lcd.setCursor(0,1);
    lcd.print("Hum: "); lcd.print(humAvg); lcd.print("%");
    Serial.print("Hum: "); Serial.println(humAvg);
}

/*!
  @brief Do something if calculated values exceed defined parameters TODO
  @param tmpAvg Average temperature calculated
  @param humAvg Average humidity calculated
  @param co2Avg Average CO2 calculated TODO
 */
void freakOut(float tmpAvg, float humAvg, float co2Avg) {
    bool tmpExceed = ((tmpAvg > MAXTEMP) or (tmpAvg < MINTEMP));
    bool humExceed = ((humAvg > MAXHUM) or (humAvg < MINHUM));
    bool co2Exceed = ((co2Avg > MAXCO2) or (co2Avg < MINCO2));
    if (tmpExceed or humExceed or co2Exceed) {
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
  @brief Log data to long-term storage TODO
  @param tmpAvg Average temperature calculated
  @param humAvg Average humidity calculated
  @param co2Avg Average CO2 calculated TODO
  @param not Time of logging event
 */
void logData(float tmpAvg, float humAvg, float co2Avg, DateTime now) {
  ;
}

void setup() {
  Serial.begin(9600);
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

  if (! rtc.initialized() || rtc.lostPower()) {
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
  
  time = millis() - time; // TODO - log this
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");

  lcd.setBacklight(WHITE);
}


void loop() {
  handleButtons();
  
  /*** Read sensor vals ***/
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  float c = 0.0; // CO2 sensor TODO
  
  if (! isnan(t)) {
    tmpVals[loopCnt % THRESHOLD] = t;
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {
    humVals[loopCnt % THRESHOLD] = h;
  } else { 
    Serial.println("Failed to read humidity");
  }

  if (! isnan(c)) {
    co2Vals[loopCnt % THRESHOLD] = h;
  } else {
    Serial.println("Failed to read CO2");
  }
  /*** End sensor read ***/
  
  /*** Do some housekeeping - maintenance, logging, and output - once the loop count threshold is reached ***/
  if (++loopCnt == THRESHOLD) {
    lcd.clear();

    // Get averages
    float tmpSum = 0.0;
    float humSum = 0.0;
    float co2Sum = 0.0;

    for (int i = 0; i < THRESHOLD; i++) {
      tmpSum += tmpVals[i];
      humSum += humVals[i];
      co2Sum += co2Vals[i];
    }
    float tmpAvg = (tmpSum/THRESHOLD)*1.8 + 32.0;
    float humAvg = humSum/THRESHOLD;
    float co2Avg = co2Sum/THRESHOLD;

    // Display/log averages
    updateDisplay(tmpAvg, humAvg, co2Avg);
    logData(tmpAvg, humAvg, co2Avg, rtc.now());

    // Do that maintenance
    enableHeater = !enableHeater;
    sht31.heater(enableHeater);
    Serial.print("Heater Enabled State: ");
    if (sht31.isHeaterEnabled())
      Serial.println("ENABLED");
    else
      Serial.println("DISABLED");

    // Reset the loop once our count gets high enough, no need to use more than a byte
    if ((loopCnt % 240) == 0) {
      loopCnt = 0;
    }
  }
  /*** End housekeeping ***/
    
  delay(100); // Read at ~10Hz

}
