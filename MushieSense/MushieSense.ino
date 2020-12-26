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
#define THRESHOLD 30 // Keep this around 30 to keep the SHT31 happy
#define MAXTEMP 90.0
#define MINTEMP 65.0
#define MAXHUM 90.0
#define MINHUM 75.0
#define MAXCO2 0.0 // TODO - set this ref. Stamets
#define MINCO2 0.0 // TODO - set this ref. Stamets

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
Adafruit_SHT31 sht31 = Adafruit_SHT31();

RTC_PCF8523 rtc;

bool enableHeater = false;

uint8_t loopCnt = 0;
uint8_t i = 0;

float tmpVals[THRESHOLD];
float humVals[THRESHOLD];

void lcdDisplay(float tmpAvg, float humAvg) {
    lcd.setCursor(0,0);
    lcd.print("Temp: "); lcd.print(tmpAvg); lcd.print(" F");
    Serial.print("Temp: "); Serial.print(tmpAvg); Serial.print("\t\t");
    lcd.setCursor(0,1);
    lcd.print("Hum: "); lcd.print(humAvg); lcd.print("%");
    Serial.print("Hum: "); Serial.println(humAvg);

    // Freak out if something's off
    if (((tmpAvg > MAXTEMP) or (tmpAvg < MINTEMP)) or ((humAvg > MAXHUM) or (humAvg < MINHUM))) {
      lcd.setBacklight(RED);
    } else {
      lcd.setBacklight(GREEN);
    }
}
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Initializing...");
  int time = millis();

  /*** Initialize variables ***/
  memset(tmpVals, 0.0, sizeof(tmpVals));
  memset(humVals, 0.0, sizeof(humVals));

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
  /*** End sensors test ***/
  
  time = millis() - time; // TODO - log this
  Serial.print("Took "); Serial.print(time); Serial.println(" ms");

  lcd.setBacklight(WHITE);
}


void loop() {
  DateTime now = rtc.now();
  
  /*** Read LCD buttons - slug code for later ***/
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
  /*** End LCD button handling ***/
  
  /*** Read sensor vals ***/
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

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
  /*** End sensor read ***/
  
  delay(100);

  
  /*** Do some housekeeping - maintenance, logging, and output - once the threshold is reached ***/
  if (++loopCnt == THRESHOLD) {
    lcd.clear();

    // Get averages
    float tmpSum = 0.0;
    float humSum = 0.0;

    for (i = 0; i < THRESHOLD; i++) {
      tmpSum += tmpVals[i];
    }
    float tmpAvg = (tmpSum/THRESHOLD)*1.8 + 32.0;

    for (i = 0; i < THRESHOLD; i++) {
      humSum += humVals[i];
    }
    float humAvg = humSum/THRESHOLD;

    // Display/log averages
    lcdDisplay(tmpAvg, humAvg);

    // Do that maintenance
    enableHeater = !enableHeater;
    sht31.heater(enableHeater);
    Serial.print("Heater Enabled State: ");
    if (sht31.isHeaterEnabled())
      Serial.println("ENABLED");
    else
      Serial.println("DISABLED");

    // Reset the loop
    loopCnt = 0;
  }
  /*** End housekeeping ***/
}
