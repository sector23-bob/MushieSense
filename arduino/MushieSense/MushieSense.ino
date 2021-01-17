#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoUniqueID.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>

#include "Adafruit_SHT31.h"

// Colors
#define RED     0x1
#define YELLOW  0x3
#define GREEN   0x2
#define TEAL    0x6
#define BLUE    0x4
#define VIOLET  0x5
#define WHITE   0x7

// Not Colors
// Control values
#define LOOP_CNT  30    // Number of loops to average values over
#define DELAY     120   // Delay per loop, in ms

#define HEATER_DELAY      30*1000       // Delay between heater cycle
#define WRITE_DELAY       5*60*1000     // Time between disk writes
#define DISPLAY_DURATION  5*1000        // Display on time on button press

// Climate control limits
#define MAXTEMP 90.0    // Set ref. Stamets
#define MINTEMP 55.0    // Set ref. Stamets
#define MAXHUM  90.0    // Set ref. Stamets
#define MINHUM  25.0    // Set ref. Stamets
#define MAXCO2  0.0     // TODO - set this ref. Stamets
#define MINCO2  0.0     // TODO - set this ref. Stamets

// RFM9x module stuff
#if defined(ESP8266)
  #define RFM95_CS    2     // "E" pin for RFM9x
  #define RFM95_RST   16    // "D" pin for RFM9x
  #define RFM95_INT   15    // "B" pin for RFM9x
#else
  #define RFM95_CS    8   // On-board Feather M0 module
  #define RFM95_RST   4   // On-board Feather M0 module
  #define RFM95_INT   3   // On-board Feather M0 module
#endif
#define RFM95_FREQ  915.0 // Frequency for radio
#define MAX_MSG     252   // Maximum message size in bytes     

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
  #define BUTTON_A  0
  #define BUTTON_B 16
  #define BUTTON_C  2
#elif defined(ESP32)
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
  #define BUTTON_A PA15
  #define BUTTON_B PC7
  #define BUTTON_C PC5
#elif defined(TEENSYDUINO)
  #define BUTTON_A  4
  #define BUTTON_B  3
  #define BUTTON_C  8
#elif defined(ARDUINO_FEATHER52832)
  #define BUTTON_A 31
  #define BUTTON_B 30
  #define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
#endif

// Unique ID
String uid;

// Temp/humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// OLED display
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// LoRa module RFM9x
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Enable/disable heater on SHT31
bool enableHeater = false;

// Timing variables
unsigned long loopTime;
unsigned long heaterLastToggled;

// Loop counter
uint8_t loopCnt = 0;

// Arrays for sensor readings
float tmpVals[LOOP_CNT];
float humVals[LOOP_CNT];
float co2Vals[LOOP_CNT];

/*!
  @brief Do something if calculated values exceed defined parameters TODO
  @param tmp Temperature to check vs. MAXTEMP, MINTEMP
  @param hum Humidity% to check vs. MAXHUM, MINHUM
  @param co2 CO2 ppm to check vs. MAXCO2, MINCO2
 */
void freakOut(float tmp, float hum, float co2) {
    bool tmpExceed = (tmp > MAXTEMP) or (tmp < MINTEMP);
    bool humExceed = (hum > MAXHUM) or (hum < MINHUM);
    bool co2Exceed = false; //(co2 > MAXCO2) or (co2 < MINCO2); // TODO
    // Do something?
}

/*!
  @brief Functionality to handle sensor errors TODO
 */
void handleSensorError() {
  ;
}

/*!
  @brief Clear the OLED display and show the text passed in
  @param text Text to be displayed
 */
void oledDisplay(String text) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(text);
  display.display();
}

/*!
  @brief Average sccumulated sensor values, send them via LoRa with device ID
 */
void doLogging() {
  /*** Get averages ***/
  float tmpSum = 0.0;
  float humSum = 0.0;
  float co2Sum = 0.0;

  for (int i = 0; i < LOOP_CNT; i++) {
    tmpSum += tmpVals[i];
    humSum += humVals[i];
    co2Sum += co2Vals[i];
  }
  float tmpAvg = tmpSum/LOOP_CNT;
  float humAvg = humSum/LOOP_CNT;
  float co2Avg = co2Sum/LOOP_CNT;

  /*** Re-zero sensor read arrays ***/
  memset(tmpVals, 0.0, sizeof(tmpVals));
  memset(humVals, 0.0, sizeof(humVals));
  memset(co2Vals, 0.0, sizeof(co2Vals));
  
  /*** Send message ***/
  // Create log string for these values
  String tmpStr = "TMP:" + String(tmpAvg) + ";HUM:" + String(humAvg);
  tmpStr += ";CO2:" + String(co2Avg) + ";ID:" + uid + '\0';
  oledDisplay(tmpStr);

  // Send LoRa message via RFM9x
  static char* message = new char[min(tmpStr.length(), MAX_MSG)];
  tmpStr.toCharArray(message, tmpStr.length());
  rf95.send((uint8_t*) message, strlen(message));
  rf95.waitPacketSent();
}

void setup() {
  Serial.begin(115200);
  while (! Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Initializing...");
  loopTime = millis();

  uid = "";
  UniqueID8dump(Serial);
  for (size_t i = 0; i < 8; i++)
  {
    uid += String(UniqueID8[i], HEX);
  }
  Serial.print("Unique ID: "); Serial.println(uid);

  /*** Initialize variables ***/
  memset(tmpVals, 0.0, sizeof(tmpVals));
  memset(humVals, 0.0, sizeof(humVals));
  memset(co2Vals, 0.0, sizeof(co2Vals));

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
  heaterLastToggled = loopTime;
  Serial.println("SHT31 OK");
  // End SHT31
  /*** End sensors test ***/
  
  /*** Set up LoRa ***/
  pinMode(RFM95_RST, OUTPUT);
  // Manual reset and test
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  Serial.println("Trying to init radio");
  while (! rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (! rf95.setFrequency(RFM95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RFM95_FREQ);
  /*** End LoRa setup ***/

  /*** OLED setup ***/
  Serial.println("OLED FeatherWing test");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  // Clear the buffer.
  display.display();

  Serial.println("IO test");

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("OLED initialized");
  display.println("Gettin' ready...");
  display.display(); // actually display all of the above

  Serial.println("OLED OK");
  /*** End OLED ***/
  
  int time = millis() - loopTime;
  Serial.print("Setup took "); Serial.print(time); Serial.println(" ms");
  Serial.flush();
}


void loop() {
  /*** Record start time for this loop ***/
  loopTime = millis();
  
  /*** Read sensor vals ***/
  float t = sht31.readTemperature()*1.8 + 32.0; // Convert to F
  float h = sht31.readHumidity();
  float c = 0.0; // CO2 sensor TODO
  
  if (! isnan(t)) {
    tmpVals[loopCnt % LOOP_CNT] = t;
  } else { 
    Serial.println("Failed to read temperature");
  }
  
  if (! isnan(h)) {
    humVals[loopCnt % LOOP_CNT] = h;
  } else { 
    Serial.println("Failed to read humidity");
  }

  if (! isnan(c)) {
    co2Vals[loopCnt % LOOP_CNT] = c;
  } else {
    Serial.println("Failed to read CO2");
  }
  /*** End sensor read ***/

  // Do we need to freak out?
  freakOut(t, h, c);

  /*** Handle buttins ***/
  if(!digitalRead(BUTTON_A)) display.print("A");
  if(!digitalRead(BUTTON_B)) display.print("B");
  if(!digitalRead(BUTTON_C)) display.print("C");
  delay(10);
  yield();
  display.display();

  /*** Do some housekeeping - maintenance, logging, and output once the loop count thresholds are reached ***/
  /*** Maintenance ***/
  // Heater
  if (loopTime - heaterLastToggled >= HEATER_DELAY) {
    Serial.println("Toggling heater");
    enableHeater = ! enableHeater;
    sht31.heater(enableHeater);
    heaterLastToggled = loopTime;
  }
  
  /*** End maintenance ***/

  /*** Logging ***/
  if (++loopCnt % LOOP_CNT == 0) {
    doLogging();

    if (loopCnt > 200) {
      loopCnt = 0;
      //Serial.printf("%f | %f | %f\n", t, h, c);
    }
  }
  /*** End logging ***/
  /*** End housekeeping ***/
  
  delay(DELAY);

}
