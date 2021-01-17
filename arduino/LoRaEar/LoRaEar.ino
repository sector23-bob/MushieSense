#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SSD1306.h>

#include <ArduinoUniqueID.h>
#include <gfxfont.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <splash.h>
#include <Wire.h>

#define DISPLAY_DURATION  5*1000        // Display activ. time on button press

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
#define RFM95_FREQ                  915.0 // Frequency for radio
#define RH_RF95_MAX_MESSAGE_LEN     252   // Maximum message size in bytes     

// LoRa module RFM9x
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// OLED display
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// Hex array
static char hex[] = "0123456789ABCDEF";

// Timing variables
unsigned long loopTime;
unsigned long displayLastActive;

bool displayActive = false;

// OLED message
char* displayMsg = {"Waiting..."};

// Unique ID
char* uid;

void setup() {
  Serial.begin(115200);
  /*
  while (! Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/
  
  Serial.println("Initializing...");
  loopTime = millis();

  uid = new char[UniqueIDsize];
  UniqueIDdump(Serial);
  for (size_t i = 0; i < UniqueIDsize; i++)
  {
    uid[(i*2) + 0] = hex[((UniqueID[i] & 0xF0) >> 4)];
    uid[(i*2) + 1] = hex[((UniqueID[i] & 0x0F) >> 0)];
    //uid[i] = hex[UniqueID[i]];
  }
  Serial.print("Unique ID: "); Serial.println(uid);

  /*** Set up and test OLED wing ***/
  Serial.println("OLED FeatherWing test");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  Serial.println("OLED begun");

  // Show buffer
  display.display();
  // Clear the buffer.
  display.clearDisplay();
  display.display();

  // Buttons
  Serial.println("IO test");
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Gettin' ready");
  display.display();
  displayLastActive = loopTime; // Initialize here
  /*** End OLED ***/

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
  display.clearDisplay();
  display.display();
}

void loop() {
  loopTime = millis();

  if ((! digitalRead(BUTTON_A)) or (! digitalRead(BUTTON_B)) or (! digitalRead(BUTTON_C))) {
    displayActive = true;
    displayLastActive = loopTime;
  }
    
  if (displayActive) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(displayMsg);
    display.display();
    if ((loopTime - displayLastActive) > DISPLAY_DURATION) {
      display.clearDisplay();
      displayActive = false;
      display.display();
    }
  }
  
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      Serial.print("got request: ");
      Serial.println((char*) buf);
      displayMsg = (char*) buf;
    }
    else {
      Serial.println("recv failed");
    }
  }
  delay(10);
  yield();
}
