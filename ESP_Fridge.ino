#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <Preferences.h>


// for ESP32, NTP should work witkput any lib.
// check: https://werner.rothschopf.net/microcontroller/202103_arduino_esp32_ntp_en.htm
#include <NTPClient.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "your_ssid"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "your_secret"
#endif

#define KEY_WIFI_SSID     "wifiSsid"
#define KEY_WIFI_PASSWORD "wifiPassword"
#define KEY_WIFI_BSSID    "wifiBssid"
//#define KEY_WIFI_CHANNEL  "wifiChannel"

#define UDP_PORT 4221

#define SECONDS 1000 // millis
#define MINUTES (60*SECONDS)
#define HOURS   (60*MINUTES)

#define TEMPERATURE_UPDATE_INTERVALL ( 1 * MINUTES)
#define HEARTBEAT_TICK_INTERVAL      (60 * SECONDS)
#define SENSOR_ALARM_INTERVAL        ( 3 * HOURS)
#define SENSOR_WARN_INTERVAL         ( 1 * HOURS)
#define CONNECTION_CHECK_INTERVAL    ( 5 * MINUTES)

#define DOOR_OPEN_WARN_TIME          (60 * SECONDS)
#define DOOR_OPEN_ALARM_TIME         (90 * SECONDS)
#define DOOR_OPENED_BEEP_TIME         10

// compressor must not be activated for at least a couple of minutes after 
// it was deactivated.
#define MOTOR_MIN_OFF_TIME           (30 * MINUTES)
#define COMPRESSOR_BEEP_ALWAYS
#define COMPRESSOR_BEEP_TIME         5


#define PIN_SENSORS 5
#define TEMPERATURE_PRECISION 11

#define WIFI_SEARCH_TIME 20 // seconds

// ouput pins:
// pin for fan motor
#define PIN_FAN 2  // connected to on-board LED, must be left floating or LOW to enter flashing mode
// checked

// pin for neopixel strip
#define PIN_PIXELS 4  // OK

// pin for heartbeat / status LED
#define PIN_LED 13

// compressor relay
#define PIN_RELAY 21 // OK

// piezo buzzer
#define PIN_BUZZER 18  // OK

// reed sensor for door
#define PIN_DOOR 23  // OK

// How many NeoPixels are attached to the Arduino?
//#define NUMPIXELS    16 // test strip
#define NUMPIXELS 27  // fridge

#define OPENED 1
#define CLOSED 0

#define PWM_CHANNEL       0
#define PWM_CHANNEL2      1
#define PWM_FREQ       8000
#define PWM_RESOLUTION    8
// minimum duty cycle that will make fan start spinning at low speed
#define PWM_MIN_CYCLE   120

// prototype
unsigned long mymillis();
long reboots;

bool compressorWasOnAtLeastOnce = false;

float hysteresis = 2;

float T_UPPER_HI = 6.0 + hysteresis/2;
float T_UPPER_LO = 6.0 - hysteresis/2;

float T_LOWER_HI = 6.0 + hysteresis/2;
float T_LOWER_LO = 6.0 - hysteresis/2;

float lowerTemp = -1;
float upperTemp = -1;

bool comprRequired = false;
bool comprActive   = false;

unsigned int lastDutyCycle = 0;

bool wasDoorClosed = 1;
long lastClosed = mymillis();
long lastTemperatureMeasurementTime = mymillis() - TEMPERATURE_UPDATE_INTERVALL - 10;

int fanDuty = -1;

unsigned long lastMotorOffTime = mymillis();
unsigned long lastMotorOnTime = 0;
unsigned long lastMotorOffDuration = 0;
unsigned long lastMotorOnDuration = 0;
unsigned long avgMotorOnDuration = 0;
unsigned long avgMotorOffDuration = 0;

unsigned long lastStatusLogged = mymillis();
unsigned long lastLoop = mymillis();
unsigned long lastTickTime = mymillis() - HEARTBEAT_TICK_INTERVAL;

unsigned long lastDoorAlarm   = mymillis();
unsigned long lastSensorAlarm = mymillis()-SENSOR_ALARM_INTERVAL;
unsigned long lastSensorWarning = mymillis()-SENSOR_WARN_INTERVAL;

unsigned long lastValidMeasureTime = mymillis();

unsigned long lastWifiSetup = mymillis();
unsigned int reconnectionAttempts = 0;

bool tempRequested = false;
bool loopCalled = false;

unsigned int heartbeatDivisor = 16;

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

// TODO put sensor mapping into preferences
int addressMapping[3] = { 0, 1, 2 };

bool doLogUDP = true;
WiFiUDP loggingUDP;

float lowerC  = DEVICE_DISCONNECTED_C;
float upperC  = DEVICE_DISCONNECTED_C;
float rearC   = DEVICE_DISCONNECTED_C;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String startTime = "?";

void logUDP(String msg);

bool isDevel() {
  String id = String(ESP.getEfuseMac());
  bool devel = id=="123538736685432";
  logUDP(String("CHIPID: ") + id + String(" DEVEL " ) + devel);
  return devel;
}

// use this method instead of millis() to allow testing if rollover is handled correctly
unsigned long mymillis() {
  unsigned long now = millis();
  //now-=60*1000;
  return now;
}

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
      beep(COMPRESSOR_BEEP_TIME);
      #else
      if (!compressorWasOnAtLeastOnce) {
        beep(COMPRESSOR_ON_BEEP_TIME);
      }
      #endif
      compressorWasOnAtLeastOnce = true;
    } else {
      #ifdef COMPRESSOR_BEEP_ALWAYS
      beep(COMPRESSOR_BEEP_TIME);
      delay(COMPRESSOR_BEEP_TIME);
      beep(COMPRESSOR_BEEP_TIME);
      #endif
    }
  }
}

#include "ota.h"
#include "wifi.h"
#include "webserver.h"
#include "sensors.h"
#include "serial.h"

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

bool measureValid(float temp) {
  // dallas sensor reports -127 resp. 85 when measurement failed
  return -80<temp && temp<80;
}

void logMotorDutyInfo() {
  logUDP(String("DEBUG: lastMotorOffDuration=") + String(lastMotorOffDuration) + " avgMotorOffDuration=" + avgMotorOffDuration);
  logUDP(String("DEBUG: lastMotorOnDuration=")  + String(lastMotorOnDuration ) + " avgMotorOnDuration="  + avgMotorOnDuration);
  if (avgMotorOnDuration+avgMotorOffDuration > 0) {
    unsigned long dutyCycle = 100*avgMotorOnDuration / (avgMotorOnDuration+avgMotorOffDuration);
    logUDP(String("DEBUG: measured motor dutyCycle=") + dutyCycle);
  }
}

void checkConnectionStatus() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (!connected) {
    const unsigned long delta = mymillis() - lastWifiSetup;
    logUDP(String("CHECKWIFI: DISCONNECTED ") + delta);    
    if (delta > CONNECTION_CHECK_INTERVAL) {
      if (reconnectionAttempts>10) {
        logUDP("ACTION: TOO MANY RECONNECTS RESTART");
        ESP.restart();
      }
      logUDP("ACTION: RECONNECT");
      const bool verbose = false;
      setupWifi(verbose);
      reconnectionAttempts++;
      lastWifiSetup = mymillis();
    }
  }
}

void measureTemperatures() {
    unsigned long uptime = mymillis();
    unsigned long delta = uptime - lastTemperatureMeasurementTime;

    if (delta > TEMPERATURE_UPDATE_INTERVALL) {
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

      if (measureValid(lowerC) && measureValid(upperC)) {
        lastValidMeasureTime = uptime;
      } else {
        if (uptime-lastSensorAlarm > SENSOR_ALARM_INTERVAL) {
          logUDP(String("ALARM: SENSOR FAILURE U ") + upperC + " L " + lowerC + " R " + rearC);
          beep(50);
          lastSensorAlarm = uptime;          
        }
      }

      if (!measureValid(rearC)) {
        if (uptime-lastSensorWarning > SENSOR_WARN_INTERVAL) {
          logUDP(String("WARN: REAR SENSOR FAILURE ") + rearC);
          lastSensorWarning = uptime;          
        }
      }

      lastTemperatureMeasurementTime = mymillis();

      beep(1);
      sensors.requestTemperatures();
      tempRequested = true;
    }
}

void setupTemperaturePresets() {
  float lowerTemp  = 3;
  float upperTemp  = 8;
  float hysteresis = 2;
  if (preferences.isKey("lowerTemp")) {    
    logUDP("KEY: lowerTemp FOUND");
    lowerTemp  = preferences.getFloat("lowerTemp");
  } else {
    logUDP("KEY: lowerTemp NOT FOUND");
  }
  if (preferences.isKey("upperTemp")) {    
    logUDP("KEY: upperTemp FOUND");
    upperTemp  = preferences.getFloat("upperTemp");
  } else {
    logUDP("KEY: upperTemp NOT FOUND");
  }
  if (preferences.isKey("hysteresis")) {    
    logUDP("KEY: hysteresis FOUND");
    hysteresis  = preferences.getFloat("hysteresis");
  } else {
    logUDP("KEY: hysteresis NOT FOUND");
  }

  T_UPPER_HI = upperTemp + hysteresis/2;
  T_UPPER_LO = upperTemp - hysteresis/2;

  T_LOWER_HI = lowerTemp + hysteresis/2;
  T_LOWER_LO = lowerTemp - hysteresis/2;

  lowerTemp = constrain(lowerTemp,   0, 30);
  upperTemp = constrain(upperTemp,   0, 30);
  hysteresis = constrain(hysteresis, 0, 10);
  
  logUDP(String("INIT: lowerTemp=") + lowerTemp);
  logUDP(String("INIT: upperTemp=") + upperTemp);
  logUDP(String("INIT: hysteresis=") + hysteresis);
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
  preferences.begin("fridge", false);

  if (preferences.isKey("reboots")) {
    reboots = preferences.getLong("reboots");
    reboots++;
  } else {
    reboots = 1;
  }
  preferences.putLong("reboots", reboots);

  WiFi.mode(WIFI_AP_STA);
  setupWifi(true);

  loggingUDP.begin(UDP_PORT);
  
  logUDP(sketch);
  logUDP(build);
  logUDP(String("WIFI: LOCAL ") + WiFi.localIP().toString() + " BSSID " + WiFi.BSSIDstr());

  setupOta();
  setupSensors();
  setupWebserver();

  setupTemperaturePresets();    
  timeClient.begin();
  timeClient.setTimeOffset(3600);

  for (int retry=0; retry<10 && !timeClient.update(); retry++) {
    timeClient.forceUpdate();
    if (!timeClient.update()) {
      delay(100);
    }
  }
  startTime = timeClient.getFormattedTime();
  logUDP(String("TIME: ") + startTime);

  setLed(255); setFan(255);
  delay(500);
  setLed(0); setFan(0);

  // get inital values ASAP
  measureTemperatures();
}

unsigned lastDebug = mymillis() - 10*1000;

void loop(void) {

  handleSerialCommands();
  server.handleClient();
  heartbeat();

  const unsigned long uptime = mymillis();
  const int door = digitalRead(PIN_DOOR);
  const bool isDoorClosed = door == CLOSED;
  const bool isDoorOpen = !isDoorClosed;
  const unsigned long doorOpenDuration = uptime - lastClosed;

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

  if (uptime - lastTickTime > HEARTBEAT_TICK_INTERVAL) {
    beep(1);
    //logUDP("TICK");
    lastTickTime = uptime;
  }

  // measure temparatures only 
  // - if door is closed to avoid delays in light animation when door opens
  // - door was open for a long time (to update temperature display)
  // - within first 10 secs after startup to get valid measures ASAP
  if (isDoorClosed || doorOpenDuration > 10000 || uptime < 10000) {
    measureTemperatures();
  }

  if (uptime - lastStatusLogged > 10000) {
    unsigned long duration = uptime - lastMotorOffTime;
    if (comprActive) {
      duration = uptime - lastMotorOnTime;
      logUDP(String("COMPRESSOR: ACTIVE ") + duration); 
    } else {
      logUDP(String("COMPRESSOR: ") + (comprRequired ? "REQUIRED " : "OFF ") + duration + "/" + MOTOR_MIN_OFF_TIME);
    } 
    logUDP(String("FAN: ") + fanDuty);
    logUDP(String("UPTIME: ") + uptime);
    logUDP(String("DOOR: ") + (isDoorOpen ? "OPEN " : "CLOSED ") + doorOpenDuration + "/" + DOOR_OPEN_WARN_TIME);
    logUDP(String("TEMP: UPPER ") + T_UPPER_LO + " " + T_UPPER_HI + " " + (upperC<T_UPPER_LO ? "LO" : upperC>T_UPPER_HI ? "HI" : "OK"));
    logUDP(String("TEMP: LOWER ") + T_LOWER_LO + " " + T_LOWER_HI + " " + (lowerC<T_LOWER_LO ? "LO" : lowerC>T_LOWER_HI ? "HI" : "OK"));
    logUDP(String("HEAP: "  ) + ESP.getFreeHeap()  + " MIN " + ESP.getMinFreeHeap() );
    logMotorDutyInfo();
    lastStatusLogged = uptime;
  }

  int fanWasDuty = fanDuty;
  if (isDoorOpen) {
    if (fanDuty > 0) {
      logUDP("ACTION: DOOR OPEN FAN OFF");
    }
    fanDuty = 0;
  } else if (upperC < T_UPPER_LO) {
    if (fanDuty > 0) {
      logUDP(String("UPPER: TEMP BELOW LOWER LIMIT ") + upperC + "<" + T_UPPER_LO);
      logUDP("ACTION: FAN OFF ");
    }
    fanDuty = 0;
  } else { // if (upperC >= T_UPPER_LO)
    const float oldDuty = fanDuty;
    // 75% of hysteresis -> start at T_UPPER_LO with 0% and increase to 66% at setpoint and furter 
    // increase to 100% beyond setpoint
    const float scale = 0.75;
    const float range = scale * hysteresis;
    const float delta = upperC - T_UPPER_LO;    

    fanDuty = 255.0 * delta/range;
    fanDuty = constrain(fanDuty, PWM_MIN_CYCLE, 255);
    if (uptime - lastDebug > 30*1000) {
      logUDP(
        String("DEBUG:") +
        String(" oldDuty=") + String(oldDuty) + 
        String(" range=") + String(scale) + String("*") + String(hysteresis) + String("=") + String(range) + 
        String(" delta=") + String(upperC)     + String("-") + String(T_UPPER_LO) + String("=") + String(delta) + 
        String(" fanDuty:=") + String("255*") + String(delta) + String("/") + String(range) + String(" => ") + fanDuty);
      lastDebug = uptime;
    }
    if (oldDuty <= 0) { // can be -1 initially
      logUDP(String("UPPER: TEMP OVER UPPER LIMIT ") + upperC + ">" + T_UPPER_HI);
      logUDP("ACTION: FAN ON");
    }
  } 

  if (fanWasDuty != fanDuty) {
    setFan(fanDuty);
    logUDP(String("ACTION: FAN DUTY CHANGED ") + fanDuty);
  }

  unsigned long deltaM = uptime - lastMotorOffTime;
  bool isWaiting = deltaM < MOTOR_MIN_OFF_TIME;

  if (lowerC > T_LOWER_HI) {
    if (!isWaiting) {
      if (!comprActive) {
        logUDP(String("ACTION: ACTIVATE COMPRESSOR ") + lowerC + ">" + T_LOWER_HI + " WAITED: " + deltaM + "/" + MOTOR_MIN_OFF_TIME);
        setCompressor(1, false);
        lastMotorOnTime = uptime;
        lastMotorOffDuration = uptime-lastMotorOffTime;
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
      lastMotorOffTime = uptime;
      lastMotorOnDuration = uptime - lastMotorOnTime;
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

  if (!isDoorClosed) {
    unsigned long delta = mymillis() - lastClosed;
    if (delta > DOOR_OPEN_WARN_TIME) {
      openWarning = true;
    }
    if (delta > DOOR_OPEN_ALARM_TIME) {
      openAlarm = true;
    }
  }

  if (isDoorOpen) {
    setFan(0);
  }

  if (isDoorClosed) {
    if (!wasDoorClosed) {
      logUDP("ACTION: CLOSED");
      setFan(255);
      logUDP("DEBUG: FAN SET TO 255");
    }    
    pixels.clear();  // Set all pixel colors to 'off'
    lastClosed = mymillis();
    wasDoorClosed = true;
  } else {
    if (wasDoorClosed) {
      logUDP("ACTION: OPENED");
      beep(DOOR_OPENED_BEEP_TIME);
      checkConnectionStatus();
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
    const bool test = isDevel();
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
      }
      if (isWaiting && !compressorWasOnAtLeastOnce) {
        pixels.setPixelColor(constrain(NUMPIXELS-1-index, 0, NUMPIXELS-1), pixels.Color(255, 0, 0));
      }
    }
    wasDoorClosed = false;
  }
  pixels.show();  // Send the updated pixel colors to the hardware.

  if (openAlarm) {
    int beep = millis() % 1000;
    if (beep < 20) {
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

  checkConnectionStatus();
}

void logUDP(String msg) {
  if (doLogUDP) {
    String formattedTime = timeClient.getFormattedTime();
    Serial.print("[");
    Serial.print(formattedTime);
    Serial.print("] ");
    Serial.println(msg);

    loggingUDP.beginPacket("255.255.255.255", UDP_PORT);
    loggingUDP.print("  [");
    loggingUDP.print(formattedTime);
    loggingUDP.print("] ");
    loggingUDP.print(msg);
    loggingUDP.endPacket();
  }
}
