#include "Adafruit_SHT31.h"

#include <Arduino.h>
#include <ArduinoUniqueID.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>

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
#define LOG_CNT 30    // Number of loops to average values over
#define DELAY   120   // Delay per loop, in ms
#define FAN_PIN 2     // We'll use pin 2 to control the fan

#define HEATER_DELAY      30*1000       // Delay between heater cycle
#define WRITE_DELAY       5*60*1000     // Time between disk writes
#define FAN_DELAY         10*60*1000    // Time between fan activation
#define FAN_DURATION      60*1000       // Duration for fan operation
#define DISPLAY_DURATION  5*1000        // Display on time on button press

// Climate control limits
#define MAXTEMP 90.0    // Set ref. Stamets
#define MINTEMP 55.0    // Set ref. Stamets
#define MAXHUM  90.0    // Set ref. Stamets
#define MINHUM  25.0    // Set ref. Stamets
#define MAXCO2  0.0     // TODO - set this ref. Stamets
#define MINCO2  0.0     // TODO - set this ref. Stamets

// RFM9x module stuff
#define RFM95_CS    2     // "E" pin for RFM9x
#define RFM95_RST   16    // "D" pin for RFM9x
#define RFM95_INT   15    // "B" pin for RFM9x
#define RFM95_FREQ  915.0 // Frequency for radio
#define MAX_MSG     252   // Maximum message size in bytes     

// Pin for fan control
#define FAN_PIN   2   // We'll control the fan with digital pin 2

// Unique ID
char* uid;

// Hex array
static char hex[] = "0123456789ABCDEF";

// Temp/humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// LoRa module RFM9x
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Enable/disable heater on SHT31
bool enableHeater = false;

// Enable/disable fan on GIO2
bool enableFan = false;

// Timing variables
unsigned long loopTime;
unsigned long heaterLastToggled;
unsigned long fanLastOn;

// Loop counter
uint8_t loopCnt = 0;

// Arrays for sensor readings
float tmpVals[LOG_CNT];
float humVals[LOG_CNT];
float co2Vals[LOG_CNT];

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
  @brief Average sccumulated sensor values, perform logging actions
  @param loopTime Time to record for logging
 */
void doLogging() {
  /*** Get averages ***/
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

  // Re-zero sensor read arrays
  memset(tmpVals, 0.0, sizeof(tmpVals));
  memset(humVals, 0.0, sizeof(humVals));
  memset(co2Vals, 0.0, sizeof(co2Vals));
  /*** End averages ***/

  /*** Perform logging actions ***/
  // Create log string for these values
  char* message = createLogString(tmpAvg, humAvg, co2Avg);

  // Send LoRa message via RFM9x
  txLoRa(message);
  /*** End logging actions ***/
}

char* createLogString(float tmp, float hum, float co2) {
  char message[1024];
  sprintf(message, "%f | %f | %f | %s", tmp, hum, co2, uid);
  Serial.println(uid);
  return message;
}

/*!
  @brief Send a message using the LoRa radio
  @param message Message to be sent
  TODO: Check message length in bytes < MAX_MSG
 */
void txLoRa(const char* message) {
  rf95.send((uint8_t*) message, strlen(message));
  rf95.waitPacketSent();
  Serial.println(message);
}

void setup() {
  Serial.begin(115200);
  while (! Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Initializing...");
  loopTime = millis();

  uid = new char[UniqueIDsize*2];
  UniqueIDdump(Serial);
  Serial.print("Serial Unique ID: ");
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    if (UniqueID[i] < 0x10) {
      Serial.print("0");
    }
    //Serial.print(UniqueID[i], HEX);
    uid[(i*2) + 0] = hex[((UniqueID[i] & 0xF0) >> 4)];
    uid[(i*2) + 1] = hex[((UniqueID[i] & 0x0F) >> 0)];
    Serial.print(" ");
  }
  Serial.println();
  Serial.println(uid);

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
  // End SHT31
  /*** End sensors test ***/

  /*** Set up fan control ***/
  pinMode(FAN_PIN, OUTPUT);
  // Manual reset and test
  digitalWrite(FAN_PIN, LOW);
  delay(10);
  digitalWrite(FAN_PIN, HIGH);
  delay(500);
  digitalWrite(FAN_PIN, LOW);
  fanLastOn = loopTime;
  Serial.println("Fan OK");
  
  /*** Set up LoRa ***/
  pinMode(RFM95_RST, OUTPUT);
  // Manual reset and test
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
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

  int time = millis() - loopTime;
  Serial.printf("Setup took %d ms\n", time);
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

  /*** Do some housekeeping - maintenance, logging, and output once the loop count thresholds are reached ***/
  /*** Maintenance ***/
  // Heater
  if (loopTime - heaterLastToggled >= HEATER_DELAY) {
    Serial.println("Toggling heater");
    enableHeater = ! enableHeater;
    sht31.heater(enableHeater);
    heaterLastToggled = loopTime;
  }
  
  // Fan
  if (! enableFan) {
    if (loopTime - fanLastOn >= FAN_DELAY) {
      Serial.println("Enabling fan");
      digitalWrite(FAN_PIN, HIGH);
      enableFan = true;
      fanLastOn = loopTime;
    }
  } else { //enableFan is true
    if (loopTime - fanLastOn >= FAN_DURATION) {
      Serial.println("Disabling fan");
      digitalWrite(FAN_PIN, LOW);
      enableFan = false;
    }
  }
  /*** End maintenance ***/

  /*** Logging ***/
  if (++loopCnt % LOG_CNT == 0) {
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
