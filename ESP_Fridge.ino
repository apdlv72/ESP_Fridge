//real:

//rear:  2843cc81e3193c07
//lower: 28d75181e3693c87
//upper: 28adce81e3f33c2c


// 0: 28d75181e3693c87 -> lower 
// 1: 28adce81e3f33c2c -> upper
// 2: 2843cc81e3193c07 -> rear


#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "your_ssid"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "your_secret"
#endif

#define UDP_PORT 4221

// Include the libraries we need
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DOOR_OPEN_WARN_TIME  60 * 1000  // 1 minute
#define DOOR_OPEN_ALARM_TIME 90 * 1000  // 1 minute
#define DOOR_OPENED_BEEP_TIME 20

#define COMPRESSOR_BEEP_ALWAYS
#define COMPRESSOR_ON_BEEP_TIME  20
#define COMPRESSOR_OFF_BEEP_TIME 10

const char FAVICON_ICO[] PROGMEM =
#include "favicon.ico.h"
;

// const char FRIDGE72_PNG[] PROGMEM =
// #include "fridge72.png.h"
// ;

const char FRIDGE192_PNG[] PROGMEM =
#include "fridge192.png.h"
;

const char MANIFEST_JSON[] PROGMEM =
#include "manifest.json.h"
;

// prototype
unsigned long mymillis();

bool compressorWasOnAtLeastOnce = false;

float T_UPPER_HI = 22.9;
float T_UPPER_LO = 22.7;

float T_LOWER_HI = 22.0;
float T_LOWER_LO = 21.7;

float lowerTemp = 3;
float upperTemp = 8;
float hysteresis = 2;

bool comprRequired = false;
bool comprActive = false;

unsigned int lastDutyCycle = 0;

bool wasClosed = 1;
long lastClosed = mymillis();
long lastTemp = mymillis() - 10000;

#define OPENED 1
#define CLOSED 0

int fanDuty = 255;

unsigned long lastMotorOff = mymillis();
unsigned long lastMotorOn = 0;
unsigned long lastMotorOffDuration = 0;
unsigned long lastMotorOnDuration = 0;
unsigned long avgMotorOnDuration = 0;
unsigned long avgMotorOffDuration = 0;

unsigned long lastUptime = mymillis();
unsigned long lastLoop = mymillis();
unsigned long lastTick = mymillis() - 60 * 1000;

bool tempRequested = false;
bool loopCalled = false;

unsigned int heartbeatDivisor = 16;

#define SECONDS 1000 // millis
#define MINUTES (60*SECONDS)

#define MOTOR_MIN_OFF_TIME (5 * MINUTES)

#define PIN_SENSORS 5
#define TEMPERATURE_PRECISION 11

#define SENSOR_ALARM_INTERVAL 3*3600*1000

#define WIFI_SEARCH_TIME 20 // seconds
// ouput pins:
// pin for fan motor
#define PIN_FAN 2  // connected to on-board LED, must be left floating or LOW to enter flashing mode

// pin for neopixel strip
#define PIN_PIXELS 4  // OK

// pin for heartbeat / status LED
#define PIN_LED 13

// compressor relay
#define PIN_RELAY 21 

// piezo buzzer
#define PIN_BUZZER 18  // OK

// reed sensor for door
#define PIN_DOOR 23  // OK


// How many NeoPixels are attached to the Arduino?
//#define NUMPIXELS    16 // test strip
#define NUMPIXELS 27  // fridge

Preferences preferences;

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_PIXELS, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 500  // Time (in milliseconds) to pause between pixels

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(PIN_SENSORS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
// DeviceAddress upperSensor = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
// DeviceAddress lowerSensor   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };
DeviceAddress addresses[3]; //upperSensor, lowerSensor, rearSensor;

int addressMapping[3] = { 0, 1, 2};

volatile bool otaOngoing = false;
int lastOtaPercent = -1;

bool doLogUDP = true;
WiFiUDP UDP;

float lowerC  = DEVICE_DISCONNECTED_C;
float upperC  = DEVICE_DISCONNECTED_C;
float rearC   = DEVICE_DISCONNECTED_C;


void logUDP(String msg);

// use this method instead of millis() to allow testing if rollover is handled correctly
unsigned long mymillis() {
  unsigned long now = millis();
  //now-=60*1000;
  return now;
}

void handleOTAStart() {
  logUDP("OTA: START");
  logUDP("ACTION: UDP OFF");
  UDP.stop();
  setCompressor(0, false);
  setLed(255);
  setFan(0);
  otaOngoing = true;
}

void handleOTAProgress(unsigned int progress, unsigned int total) {
  int percent = (progress / (total / 100));
  if (percent != lastOtaPercent) {
    // UDP is off here
    Serial.printf("OTA: %u%%\n", percent);
    int state = percent % 5;
    digitalWrite(PIN_BUZZER, state == 0 ? LOW : HIGH);    
  } else {
    digitalWrite(PIN_BUZZER, HIGH);    
  }
  setLed(progress%255);  
  lastOtaPercent = percent;
}

void handleOTAEnd() {
  otaOngoing = false;
  Serial.println("OTA: DONE");
  ESP.restart();
}

unsigned int PWM_CHANNEL = 0;
unsigned int PWM_CHANNEL2 = 1;
unsigned int PWM_FREQ = 8000;
unsigned int PWM_RESOLUTION = 8; // bit
unsigned int PWM_MIN_CYCLE = 120;

void setFan(int dutyCycle) {
  lastDutyCycle = dutyCycle;
  ledcWrite(PWM_CHANNEL, dutyCycle);
}

void setLed(int dutyCycle) {
  ledcWrite(PWM_CHANNEL2, dutyCycle);
}

void setCompressor(int onOff, bool silent) {
  digitalWrite(PIN_RELAY, onOff==0 ? LOW : HIGH);
  if (!silent) {
    if (onOff) {
      #ifdef COMPRESSOR_BEEP_ALWAYS
      beep(COMPRESSOR_ON_BEEP_TIME);
      #else
      if (!compressorWasOnAtLeastOnce) {
        beep(COMPRESSOR_ON_BEEP_TIME);
      }
      #endif
      compressorWasOnAtLeastOnce = true;
    } else {
      #ifdef COMPRESSOR_BEEP_ALWAYS
      beep(COMPRESSOR_OFF_BEEP_TIME);
      #endif
    }
  }
}

unsigned long lastDoorAlarm   = mymillis();
unsigned long lastSensorAlarm = mymillis()-SENSOR_ALARM_INTERVAL;

void swapBytes(uint8_t * a, uint8_t *b, unsigned int count) {
  for (unsigned int i=0; i<count; i++) {
    uint8_t c = *a;
    *a = *b;
    *b = c;
    a++;
    b++;
  }
}

void setupSensors() {
  sensors.begin();

  // locate devices on the bus
  logUDP(String("SENSORS: FOUND ") + sensors.getDeviceCount() + " DEVICES");

  // report parasite power requirements
  logUDP(String("PARSITE: ") + (sensors.isParasitePowerMode() ? "ON" : "OFF"));

  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  //
  // method 1: by index
  if (!sensors.getAddress(addresses[0], 0)) {
    logUDP("ERROR: DEVICE 0 NO ADDR");
    bzero(addresses[0], sizeof(addresses[0]));
  }
  if (!sensors.getAddress(addresses[1], 1)) {
    logUDP("ERROR: DEVICE 1 NO ADDR");
    bzero(addresses[1], sizeof(addresses[1]));
  }
  if (!sensors.getAddress(addresses[2],  2)) {
    logUDP("ERROR: DEVICE 2 NO ADDR");
    bzero(addresses[2], sizeof(addresses[2]));
  }

  // set the resolution to x bit per device
  sensors.setResolution(addresses[0], TEMPERATURE_PRECISION);
  sensors.setResolution(addresses[1], TEMPERATURE_PRECISION);
  sensors.setResolution(addresses[2], TEMPERATURE_PRECISION);

  // sort device addresses in descending order (highest first) so empty addresses 
  const int size = sizeof(DeviceAddress);
  if (memcmp(addresses[0],  addresses[1], size) < 0) {
    swapBytes(addresses[0], addresses[1], size);
  }
  if (memcmp(addresses[1],  addresses[2], size) < 0) {
    swapBytes(addresses[1], addresses[2], size);
  }
  if (memcmp(addresses[0],  addresses[1], size) < 0) {
    swapBytes(addresses[0], addresses[1], size);
  }
  
  // show the addresses we found on the bus
  logUDP(String("SENSORS: 0: ADDR: ") + formatAddress(addresses[0]) + " PREC: " + sensors.getResolution(addresses[0]));
  logUDP(String("SENSORS: 1: ADDR: ") + formatAddress(addresses[1]) + " PREC: " + sensors.getResolution(addresses[1]));
  logUDP(String("SENSORS: 2: ADDR: ") + formatAddress(addresses[2]) + " PREC: " + sensors.getResolution(addresses[2]));
}

unsigned long lastWifiSetup = mymillis();

void setupWifi() {
  // https://randomnerdtutorials.com/esp32-set-custom-hostname-arduino/
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("ESP32_Fridge");

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("WIFI: CONNECTING");

  digitalWrite(PIN_BUZZER, LOW);
  delay(100);
  for (int retries = 0; WiFi.status() != WL_CONNECTED && retries < 10*WIFI_SEARCH_TIME; retries++) {
    setLed(0);
    digitalWrite(PIN_BUZZER, LOW);
    delay(10);

    setLed(255);
    digitalWrite(PIN_BUZZER, HIGH);
    delay(90);

    int interval = retries % (2*NUMPIXELS); // 0, ..., 23
    int index = (interval<NUMPIXELS) ? interval : 2*NUMPIXELS-1-interval;
    pixels.clear();
    pixels.setPixelColor(constrain(index, 0, NUMPIXELS-1), pixels.Color(0, 0, 255));
    pixels.show();
  }
  setLed(0);
  digitalWrite(PIN_BUZZER, HIGH);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI: FAILED");
    pixels.clear();
    for (int i=0; i<NUMPIXELS; i++) {
      pixels.setPixelColor(constrain(i, 0, NUMPIXELS-1), pixels.Color(20, 0, 0));
    }
    pixels.show();
    digitalWrite(PIN_BUZZER, LOW);
    delay(1000);
    digitalWrite(PIN_BUZZER, HIGH);
    //ESP.restart();
  } else {
    for (int i = 0; i < 3; i++) {
      setLed(16); digitalWrite(PIN_BUZZER, LOW);
      delay(100);
      setLed(0); digitalWrite(PIN_BUZZER, HIGH);
      delay(100);
    }
  }
  lastWifiSetup = mymillis();
}

WebServer server(80);

void handleNotFound() {
  logUDP("DEBUG: handleNotFound");
  server.send(404, "text/plain", "File Not Found\n\n");
}

void serverIndex() {
  logUDP("DEBUG: serverIndex");
  unsigned long uptime = mymillis();

  unsigned long motorOnTime = uptime-lastMotorOn;

  int door = digitalRead(PIN_DOOR);
  bool isOpen = (door != CLOSED);
  unsigned long deltaDoor = uptime - lastClosed;
  String commit = String("$Id$");
  String hash = commit.substring(5).substring(0, 7);

  String html = String("<!DOCTYPE html PUBLIC \"-//WAPFORUM//DTD XHTML Mobile 1.0//EN\" \"http://www.wapforum.org/DTD/xhtml-mobile10.dtd\">\n");
  html += "<html>\n";
  html += "<head>\n";
  // https://developer.chrome.com/multidevice/android/installtohomescreen
  html += "<meta charset='utf-8'>\n";
  html += "<meta name=\"viewport\" content=\"width=device-width\">\n";
  html += "<meta name=\"mobile-web-app-capable\" content=\"yes\">\n";
  html += "<link rel=\"shortcut icon\" type=\"image/x-icon\" href=\"/favicon.ico\">\n";
  html += "<link rel=\"manifest\" href=\"/manifest.json\">\n";
  html += "<title>Fridge</title>\n";
  html += "<style type=\"text/css\">\n";
  html += "body { font-family: verdana, arial, sans-serif; background-color: #a0a0ff; }";
  html += "</style>";
  html += "</head>\n";
  html += "<body>\n";
  html += "<form action=\"/set\">\n";
  html += "<table>\n";
  html += "<tr><td>Build:</td><td>" __DATE__ " " __TIME__ " </td></tr>\n";
  html += "<tr><td>Commit:</td><td>" + hash + "</td></tr>\n";
  html += "<tr><td>Uptime:</td><td>" + String(uptime) + " ms</td></tr>\n";
  html += "<tr><td>Upper temp:</td><td>" + String(upperC) + " &deg;C</td></tr>\n";
  html += "<tr><td>Lower temp:</td><td>" + String(lowerC) + " &deg;C</td></tr>\n";
  html += "<tr><td>Rear  temp:</td><td>" + String(rearC)  + " &deg;C</td></tr>\n";

  html += "<tr><td>Compressor:</td><td>";
  if (comprActive) {
    unsigned duration = uptime-lastMotorOn;
    html += String("ACTIVE ") + duration; 
  } else {
    unsigned long duration = uptime - lastMotorOff;
    html += String(comprRequired ? "REQUIRED " : "OFF ") + duration + "/" + String(MOTOR_MIN_OFF_TIME);
  } 
  html += "</td></tr>\n";
  
  html += "<tr><td>Door:</td><td>";
  if (isOpen) {
      html += "OPEN ";
      html += deltaDoor;
      html += " ms";
  } else {
      html += "CLOSED";
  }
  html += "</td></tr>\n";

  float percentage = (100.0*lastDutyCycle)/255;
  html += "<tr><td>Fan</td><td>" + String(percentage)  + " %</td></tr>\n";
  html += "<tr><td>Upper setpoint:</td><td><input type=\"text\" name=\"upper\" value=\""  + String((T_UPPER_HI+T_UPPER_LO)/2) + "\" />&deg;C</td></tr>\n";
  html += "<tr><td>Lower setpoint:</td><td><input type=\"text\" name=\"lower\" value=\""  + String((T_LOWER_HI+T_LOWER_LO)/2) + "\" />&deg;C</td></tr>\n";
  html += "<tr><td>Hysteresis:    </td><td><input type=\"text\" name=\"hyster\" value=\"" + String(hysteresis) + "\" />&deg;C</td></tr>\n";
  html += "<tr><td><input type=\"submit\" name=\"submit\" value=\"Submit\" /></td></tr>\n";
  html += "</table>\n";
  html += "</form>\n";
  html += "<br/><br/>\n";
  html += "<a href=\"/reset\">[RESET]</a>\n";
  html += "\n</body>\n</html>\n";
  server.send(200, "text/html", html);
}

void serverRedirect(unsigned int seconds) {
  logUDP("DEBUG: serverRedirect");
  String html = String("<html>\n");
  html += "<head>\n";
  html += "<meta http-equiv=\"refresh\" content=\"" + String(seconds) + "; URL=/\">\n";
  html += "</head>\n";
  html += "<body style=\"font-family: verdana, arial, sans-serif; background-color: #a0ffa0;\">\n";
  html += "<h1><center>OK</center></h1>\n";
  html += "</body>\n";
  html += "</html>\n";
  server.send(200, "text/html", html);
}

void serverReset() {
  logUDP("DEBUG: serverReset");
  serverRedirect(5);
  logUDP("ACTION: SERVER RESET");
  beep(30);
  ESP.restart();
}

void serverSet() {
  logUDP("DEBUG: serverSet");
  String upper = server.arg(0);
  String lower = server.arg(1);
  String hyster = server.arg(2);

  upperTemp  = constrain(upper.toFloat(),  0, 30);
  lowerTemp  = constrain(lower.toFloat(),  0, 30);
  hysteresis = constrain(hyster.toFloat(), 0, 10);

  preferences.putFloat("upperTemp", upperTemp);
  preferences.putFloat("lowerTemp", lowerTemp);
  preferences.putFloat("hysteresis", hysteresis);

  T_UPPER_HI = upperTemp + hysteresis/2;
  T_UPPER_LO = upperTemp - hysteresis/2;
  T_LOWER_HI = lowerTemp + hysteresis/2;
  T_LOWER_LO = lowerTemp - hysteresis/2;

  logUDP(String("SERVER: LOWER [") + T_LOWER_LO + ", " + T_LOWER_HI + "]");
  logUDP(String("SERVER: UPPER [") + T_UPPER_LO + ", " + T_UPPER_HI + "]");
  serverRedirect(2);
  beep(30);
}

const char PGM_CT_APPL_JSON[] PROGMEM = "application/json";

void sendBinary(PGM_P src, size_t contentLength, const char *contentType) {
  logUDP("DEBUG: sendBinary");
  WiFiClient client = server.client();
  String head
    = String("HTTP/1.0 200 OK\r\n") +
      "Content-Type: "   + contentType + "\r\n"
      "Content-Length: " + contentLength + "\r\n"
      "Connection: close\r\n"
      "\r\n";
  client.write(head.c_str(), head.length());

  char buffer[256];
  const unsigned int size = sizeof(buffer);

  unsigned int offs = 0;
  unsigned int left = contentLength;
  do {
    unsigned int count = left<size ? left : size;
    memcpy_P(buffer, src+offs, count);
    client.write(buffer, count);
    client.flush();
    offs+=count;
    left-=count;
  } while (left>0);
  client.stop();
}

// void handleHomeIcon72() {
//    logUDP("DEBUG: handleHomeIcon72");
//  	 sendBinary(FRIDGE72_PNG, sizeof(FRIDGE72_PNG), "image/png");
// }

void handleHomeIcon192() {
   logUDP("DEBUG: handleHomeIcon192");
 	 sendBinary(FRIDGE192_PNG, sizeof(FRIDGE192_PNG), "image/png");
}

void handleFavicon() {
  logUDP("DEBUG: handleFavicon");
  sendBinary(FAVICON_ICO, sizeof(FAVICON_ICO), "image/x-icon");
}

void handleManifest() {
  logUDP("DEBUG: handleManifest");
  server.send_P(200, PGM_CT_APPL_JSON, MANIFEST_JSON);
}

void setupWebserver() {
  server. on("/", serverIndex);
  server. on("/index.html", serverIndex);
  server. on("/set", serverSet);
  server. on("/reset", serverReset);

  server.on("/manifest.json", handleManifest);
  server.on("/favicon.ico",   handleFavicon);
  // server.on("/fridge72.png",  handleHomeIcon72);
  server.on("/fridge192.png", handleHomeIcon192);

  server.onNotFound(handleNotFound);
  server.begin();
}

void setup(void) {
  const char * sketch = "SKETCH: ESP_Fridge started";
  const char * build  = "BUILD: " __DATE__ " " __TIME__;

  Serial.begin(115200);
  Serial.println(sketch);
  Serial.println(build);
  //testMillisOverflow();

  digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_RELAY, OUTPUT);

  digitalWrite(PIN_LED, HIGH);
  pinMode(PIN_LED, OUTPUT);  
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_LED, PWM_CHANNEL2);

  pinMode(PIN_DOOR, INPUT_PULLUP);

  digitalWrite(PIN_FAN, LOW);  // off
  pinMode(PIN_FAN, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_FAN, PWM_CHANNEL);
  setFan(0);

  digitalWrite(PIN_BUZZER, HIGH);
  pinMode(PIN_BUZZER, OUTPUT);

  pixels.begin();

  WiFi.mode(WIFI_STA);
  setupWifi();

  logUDP(sketch);
  logUDP(build);
  UDP.begin(UDP_PORT);

  logUDP(String("WIFI: LOCAL: ") + WiFi.localIP().toString() + " BSSID: " + WiFi.BSSIDstr());
  logUDP("FRITZBOX7490: 34:31:C4:B8:31:33");
  logUDP("FRITZBOX7590: B0:F2:08:1D:10:1D");

  logUDP("INIT_OTA: START");
  ArduinoOTA.onStart(handleOTAStart);
  ArduinoOTA.onProgress(handleOTAProgress);
  ArduinoOTA.onEnd(handleOTAEnd);

  ArduinoOTA.setHostname("ESP_Fridge_OTA");
  //ArduinoOTA.setPassword("4711");
  ArduinoOTA.begin();
  logUDP("INIT_OTA: DONE");

  setupSensors();
  setupWebserver();

  preferences.begin("fridge", false);

  float lowerTemp  = 3;
  float upperTemp  = 8;
  float hysteresis = 2;
  if (preferences.isKey("lowerTemp")) {    
    lowerTemp  = preferences.getFloat("lowerTemp");
  }
  if (preferences.isKey("upperTemp")) {    
    upperTemp  = preferences.getFloat("upperTemp");
  }
  if (preferences.isKey("hysteresis")) {    
    hysteresis  = preferences.getFloat("hysteresis");
  }

  T_UPPER_HI = upperTemp + hysteresis/2;
  T_UPPER_LO = upperTemp - hysteresis/2;

  T_LOWER_HI = lowerTemp + hysteresis/2;
  T_LOWER_LO = lowerTemp - hysteresis/2;

  logUDP(String("INIT: lowerTemp=") + lowerTemp);
  logUDP(String("INIT: upperTemp=") + upperTemp);
  logUDP(String("INIT: hysteresis=") + hysteresis);

  lowerTemp = constrain(lowerTemp,   0, 30);
  upperTemp = constrain(upperTemp,   0, 30);
  hysteresis = constrain(hysteresis, 0, 10);
  
  logUDP(String("INIT: lowerTemp=") + lowerTemp);
  logUDP(String("INIT: upperTemp=") + upperTemp);
  logUDP(String("INIT: hysteresis=") + hysteresis);
  
  setLed(255); setFan(255);
  delay(500);
  setLed(0); setFan(0);
}

// function to print a device address
String formatAddress(DeviceAddress deviceAddress) {
  String rtv = "";
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) rtv+= "0";
    rtv += String(deviceAddress[i], HEX);
  }
  return rtv;
}

// function to print the temperature for a device
float readTemperature(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return DEVICE_DISCONNECTED_C;
  }
  return tempC;
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress) {
  logUDP(String("Resolution: ") + sensors.getResolution(deviceAddress));
}

// main function to print information about a device
float printData(DeviceAddress deviceAddress) {
  float tempC = readTemperature(deviceAddress);
  logUDP(String("SENSOR: ") +  formatAddress(deviceAddress) + " C: " + tempC);
  return tempC;
}

void beep(long ms) {
  digitalWrite(PIN_BUZZER, LOW);
  delay(ms);
  digitalWrite(PIN_BUZZER, HIGH);
}

void heartbeat() {
  bool disconnected = (WiFi.status() != WL_CONNECTED);
  if (disconnected) {
    heartbeatDivisor = comprActive ? 1 : 2;
  } else {
    heartbeatDivisor = comprActive ? 8 : 16;
  }

  unsigned long uptime = mymillis();
  unsigned int seconds = (8*uptime/heartbeatDivisor)%2500;
  unsigned int duty = 0;
  if (seconds<256) {
    duty = seconds;
  } else if (seconds<512) {
    duty = 511-seconds;
  } 
  duty /= 10;  // 10% brightness
  setLed(duty);
}

void handleSerialDebug() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    Serial.print("ECHO: "); Serial.println(data);
    if (data=="RST") {
      pixels.clear();
      pixels.show();
      ESP.restart();
    } else if (data=="DIS") {
      WiFi.disconnect(false, false);
    } else if (data=="EN") {
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    } else if (data=="HD1") {
      heartbeatDivisor = 1;
    } else if (data=="HD2") {
      heartbeatDivisor = 2;
    } else if (data=="HD4") {
      heartbeatDivisor = 4;
    } else if (data=="HD8") {
      heartbeatDivisor = 8;
    } else if (data=="HD16") {
      heartbeatDivisor = 16;
    } else if (data=="HD32") {
      heartbeatDivisor = 32;
    } else if (data=="CON"){
      setCompressor(1, true);
    } else if (data=="COFF"){
      setCompressor(0, true);
    } else if (data=="FON"){
      int door = digitalRead(PIN_DOOR);
      bool isClosed = door == CLOSED;
      bool isOpen = !isClosed;
      if (isOpen) {
        logUDP("ERROR: CMD DENIED DOOR IS OPEN");
        logUDP("ERROR: CMD DENIED DOOR IS OPEN");
        logUDP("ERROR: CMD DENIED DOOR IS OPEN");
      }
      setFan(255);
    } else if (data=="FOFF") {
      setFan(0);
    } else if (data=="F0") {
      setFan(0);
    } else if (data=="F1") {
      setFan(1);
    } else if (data=="F80") {
      setFan(80);
    } else if (data=="F90") {
      setFan(90);
    } else if (data=="F100") {
      setFan(100);
    } else if (data=="F120") {
      setFan(120);
    } else if (data=="F150") {      
      setFan(150);
    } else if (data=="F200") {      
      setFan(200);
    } else if (data=="F255") {      
      setFan(255);
    } else if (data=="F512") {      
      setFan(512);
    } else {
      Serial.print("ERROR: "); Serial.println(data);
    }
  }
}

unsigned int reconnectionAttempts = 0;

void logMotorDutyInfo() {
  logUDP(String("DEBUG: lastMotorOffDuration=") + lastMotorOffDuration);
  logUDP(String("DEBUG: avgMotorOffDuration=")  + avgMotorOffDuration);
  logUDP(String("DEBUG: lastMotorOnDuration=")  + lastMotorOnDuration);
  logUDP(String("DEBUG: avgMotorOnDuration=")   + avgMotorOnDuration);
  if (avgMotorOnDuration+avgMotorOffDuration > 0) {
    unsigned long dutyCycle = 100*avgMotorOnDuration / (avgMotorOnDuration+avgMotorOffDuration);
    logUDP(String("DEBUG: dutyCycle=") + dutyCycle);
  }
}

void loop(void) {

  handleSerialDebug();
  server.handleClient();

  unsigned long uptime = mymillis();
  heartbeat();
  int door = digitalRead(PIN_DOOR);
  bool isClosed = door == CLOSED;
  bool isOpen = !isClosed;
  unsigned long deltaDoor = uptime - lastClosed;

  if (!loopCalled) {
    logUDP(String("START: ") + uptime);
    loopCalled = true;
  }

  ArduinoOTA.handle();
  if (uptime - lastLoop < 20) {
    return;
  }
  lastLoop = uptime;
  if (otaOngoing) {
    return;
  }

  if (uptime - lastTick > 10 * 1000) {
    beep(1);
    //logUDP("TICK");
    lastTick = uptime;
  }

  unsigned long delta = deltaDoor;
  // measure temparatures only if door is closed or was open for a long time
  // to avoid delays in the light animation when door is opened
  // also measure within the forst 10 seconds of uptime to get first values asap
  if (isClosed || delta > 10000 || uptime < 10000) {
    if (uptime - lastTemp > 10000) {
      if (!tempRequested) {
        digitalWrite(PIN_BUZZER, HIGH);  // avoid long beeps
        unsigned long t0 = mymillis();
        sensors.requestTemperatures();
        unsigned long t1 = mymillis();
        logUDP(String("MEASURE: DURATION ") + (t1-t0));
        tempRequested = true;
      }

      lowerC  = printData(addresses[addressMapping[0]]);        
      upperC  = printData(addresses[addressMapping[1]]);
      rearC   = printData(addresses[addressMapping[2]]);

      if (upperC<100 || lowerC<100 || rearC<100) {
        if (uptime-lastSensorAlarm > SENSOR_ALARM_INTERVAL) {
          logUDP(String("ALARM: SENSOR FAILURE U ") + upperC + " L " + lowerC + " R " + rearC);
          beep(50);
          lastSensorAlarm = uptime;          
        }
      }

      lastTemp = mymillis();

      digitalWrite(PIN_BUZZER, HIGH); // avoid long beeps
      sensors.requestTemperatures();
      tempRequested = true;
    }
  }

  if (uptime - lastUptime > 10000) {
    unsigned long duration = uptime - lastMotorOff;
    if (comprActive) {
      duration = uptime-lastMotorOn;
      logUDP(String("COMPRESSOR: ACTIVE ") + duration); 
    } else {
      logUDP(String("COMPRESSOR: ") + (comprRequired ? "REQUIRED " : "OFF ") + duration + "/" + MOTOR_MIN_OFF_TIME);
    } 
    logUDP(String("FAN: ") + fanDuty);
    logUDP(String("UPTIME: ") + uptime);
    logUDP(String("DOOR: ") + (isOpen ? "OPEN " : "CLOSED ") + deltaDoor + "/" + DOOR_OPEN_WARN_TIME);
    logUDP(String("TEMP: UPPER ") + T_UPPER_LO + " " + T_UPPER_HI + " " + (upperC<T_UPPER_LO ? "LO" : upperC>T_UPPER_HI ? "HI" : "OK"));
    logUDP(String("TEMP: LOWER ") + T_LOWER_LO + " " + T_LOWER_HI + " " + (lowerC<T_LOWER_LO ? "LO" : lowerC>T_LOWER_HI ? "HI" : "OK"));
    logUDP(String("HEAP: "  ) + ESP.getFreeHeap()  + " MIN " + ESP.getMinFreeHeap() );
    logMotorDutyInfo();
    lastUptime = uptime;
  }

  int fanWasDuty = fanDuty;
  if (isOpen) {
    if (fanDuty > 0) {
      logUDP("ACTION: DOOR OPEN FAN OFF");
    }
    fanDuty = 0;
  } else if (upperC > T_UPPER_HI) {
    if (fanDuty == 0) {
      logUDP(String("ACTION: UPPER TOO HIGH FAN ON ") + upperC + ">" + T_UPPER_HI);
    }
    float deltaT = upperC - T_UPPER_HI;
    fanDuty = round(100 * deltaT);
    if (fanDuty < PWM_MIN_CYCLE) fanDuty = PWM_MIN_CYCLE;
    if (fanDuty > 255) fanDuty = 255;
  } else if (upperC < T_UPPER_LO) {
    if (fanDuty > 0) {
      logUDP(String("ACTION: UPPER OK FAN OFF ") + upperC + "<" + T_UPPER_LO);
    }
    fanDuty = 0;
  }

  if (fanWasDuty != fanDuty) {
    setFan(fanDuty);
    logUDP(String("ACTION: FAN DUTY ") + fanDuty);
  }

  unsigned long deltaM = uptime - lastMotorOff;
  bool isWaiting = deltaM < MOTOR_MIN_OFF_TIME;

  if (lowerC > T_LOWER_LO) {
    if (!isWaiting) {
      if (!comprActive) {
        logUDP(String("ACTION: ACTIVATE COMPRESSOR ") + lowerC + ">" + T_LOWER_HI + " WAITED: " + deltaM + "/" + MOTOR_MIN_OFF_TIME);
        setCompressor(1, false);
        lastMotorOn = uptime;
        lastMotorOffDuration = uptime-lastMotorOff;
        if (avgMotorOffDuration==0) {
          avgMotorOffDuration = lastMotorOffDuration;
        } else {
          avgMotorOffDuration = (2*lastMotorOffDuration + avgMotorOffDuration)/3;
        }
        logMotorDutyInfo();        
        comprActive = true;
      }
    } else if (!comprRequired) {
      logUDP(String("ACTION: COMPRESSOR REQUIRED ") + lowerC + ">" + T_LOWER_HI + " WAITED: " + deltaM + "/" + MOTOR_MIN_OFF_TIME);
    }
    comprRequired = true; 
  } else if (lowerC < T_LOWER_LO) {
    if (comprRequired) {
      logUDP(String("ACTION: DEACTIVATE COMPRESSOR ") + lowerC + "<" + T_LOWER_LO);
      setCompressor(0, false);
      lastMotorOff = uptime;
      lastMotorOnDuration = uptime-lastMotorOn;
      if (avgMotorOnDuration==0) {
        avgMotorOnDuration = lastMotorOnDuration;
      } else {
        avgMotorOnDuration = (2*lastMotorOnDuration + avgMotorOnDuration)/3;
      }
      logMotorDutyInfo();        
    }
    comprRequired = comprActive = false;
  }

  bool openWarning = false;
  bool openAlarm = false;

  if (!isClosed) {
    unsigned long delta = mymillis() - lastClosed;
    if (delta > DOOR_OPEN_WARN_TIME) {
      openWarning = true;
    }
    if (delta > DOOR_OPEN_ALARM_TIME) {
      openAlarm = true;
    }
  }

  if (isOpen) {
    setFan(0);
  }

  if (isClosed) {
    if (!wasClosed) {
      logUDP("ACTION: CLOSED");
      setFan(255);
      logUDP("DEBUG: FAN SET TO 255");
    }    
    pixels.clear();  // Set all pixel colors to 'off'
    lastClosed = mymillis();
    wasClosed = true;
  } else {
    if (wasClosed) {
      logUDP("ACTION: OPENED");
      beep(DOOR_OPENED_BEEP_TIME);
    }
    unsigned long delta = mymillis() - lastClosed;
    long count = (delta / 4000.0) * NUMPIXELS;
    for (int i = 0; i < NUMPIXELS; i++) {
      if (i < count) {
        int blink = mymillis() % 1000;
        if (openWarning) {
          pixels.setPixelColor(i, pixels.Color(blink < 500 ? 240 : 0, 0, 0));
        } else {
          pixels.setPixelColor(i, pixels.Color(255, 220, 80));
        }
      } else {
        pixels.setPixelColor(i, pixels.Color(0, 0, 10));
      }
    }

    float upper = upperC;
    float lower = lowerC;
    bool test = true;
    if (test) {
      upper /= 5;
      lower /= 5;
    }

    int upperIdx = constrain(-1, upper - 1, NUMPIXELS - 1);  // 0°, 1°, ..., 12° -> -1, 0, ...,  11
    int lowerIdx = constrain(-1, lower - 1, NUMPIXELS - 1);  // 0°, 1°, ..., 12° -> -1, 0, ...,  11
    lowerIdx = NUMPIXELS - 1 - lowerIdx;                     // 11+1, 11-0, ..., 11-11 = 12, 11, ..., 0

    if (upperIdx > NUMPIXELS - 1) upperIdx = NUMPIXELS - 1;
    if (lowerIdx < 0) lowerIdx = 0;

    int blue = upperIdx == lowerIdx ? 50 : 0;
    if (0 <= upperIdx && upperIdx < NUMPIXELS) pixels.setPixelColor(upperIdx, pixels.Color(0, 50, blue));  // yields cyan if both, blue and green set
    if (0 <= lowerIdx && lowerIdx < NUMPIXELS) pixels.setPixelColor(lowerIdx, pixels.Color(0, 0, 50));

    bool disconnected = (WiFi.status() != WL_CONNECTED);    
    if (!disconnected) {
      reconnectionAttempts=0;
    }

    if (disconnected || isWaiting) {
      int interval = (uptime/50) % (2*NUMPIXELS); // 0, ..., 23
      int index = (interval<NUMPIXELS) ? interval : 2*NUMPIXELS-1-interval; // 23-12, ..., 23-23 => 11, ..., 0
      if (disconnected) {
        pixels.setPixelColor(constrain(index, 0, NUMPIXELS-1), pixels.Color(0, 0, 255));
        if (uptime - lastWifiSetup > 5*60*1000) {
          if (reconnectionAttempts>10) {
            logUDP("ACTION: TOO MANY RECONNECTS RESTART");
            ESP.restart();
          }
          logUDP("ACTION: RECONNECT");
          setupWifi();
          reconnectionAttempts++;
          lastWifiSetup = uptime;
        }
      }
      if (isWaiting && !compressorWasOnAtLeastOnce) {
        pixels.setPixelColor(constrain(NUMPIXELS-index, 0, NUMPIXELS-1), pixels.Color(255, 0, 0));
      }
    }
    wasClosed = false;
  }
  pixels.show();  // Send the updated pixel colors to the hardware.

  if (openAlarm) {
    int beep = millis() % 1000;
    if (beep < 100) {
      digitalWrite(PIN_BUZZER, LOW);
      lastDoorAlarm = uptime;
    } else {
      digitalWrite(PIN_BUZZER, HIGH);
    }
    if (uptime-lastDoorAlarm > 5000) {
      logUDP("ALARM: DOOR OPEN");
    }
  } else {
    digitalWrite(PIN_BUZZER, HIGH);
  }
}

void logUDP(String msg) {
  if (doLogUDP) {
    Serial.println(msg);
    UDP.beginPacket("255.255.255.255", UDP_PORT);
    UDP.print("  ");
    UDP.print(msg);
    UDP.endPacket();
  }
}
