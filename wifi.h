#include "IPAddress.h"
#ifndef wifi_h
#define wifi_h

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "your_ssid"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "your_secret"
#endif

#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS "your_ap_secret"
#endif

#define KEY_WIFI_SSID     "wifiSsid"
#define KEY_WIFI_PASSWORD "wifiPassword"
#define KEY_WIFI_BSSID    "wifiBssid"

String wifi_ssid     = WIFI_SSID;
String wifi_password = WIFI_PASS;
String wifi_bssid    = "";

bool softApStarted = false;
bool restartApOnReconnect = false;

uint8_t strToHex(const char str[]) {
  return (uint8_t) strtol(str, 0, 16);
}

bool parseBssid(String bssidStr, uint8_t buff[6]) {
  if (bssidStr.length()!=17) {
    return false;
  }
  for (int i=0; i<6; i++) {
    buff[i] = strToHex(bssidStr.substring(3*i, 3*i+2).c_str());
  }
  return true;
}

void animateSuccess() {
  for (int i = 0; i < 3; i++) {
    setLed(16); digitalWrite(PIN_BUZZER, LOW);
    delay(100);
    setLed(0); digitalWrite(PIN_BUZZER, HIGH);
    delay(100);
  }
}

void animateFailure() {
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
}

void setupWifi(boolean verbose) {

  if (preferences.isKey(KEY_WIFI_SSID)) {    
    wifi_ssid  = preferences.getString(KEY_WIFI_SSID);
  } else {
    logUDP(String("KEY: " KEY_WIFI_SSID) + " NOT FOUND");
  }
  if (preferences.isKey(KEY_WIFI_BSSID)) {    
    wifi_bssid  = preferences.getString(KEY_WIFI_BSSID);
  } else {
    logUDP(String("KEY: " KEY_WIFI_BSSID) + " NOT FOUND");
  }
  if (preferences.isKey(KEY_WIFI_PASSWORD)) {    
    wifi_password  = preferences.getString(KEY_WIFI_PASSWORD);
  } else {
    logUDP(String("KEY: " KEY_WIFI_PASSWORD) + " NOT FOUND");
  }

  logUDP(String("WIFI: SSID '") + wifi_ssid + "'");
  logUDP(String("WIFI: BSSID '") + wifi_bssid + "'");

  uint8_t bssid[6];
  bool bssidValid = parseBssid(wifi_bssid, bssid);
  //logUDP(String("DEBUG: bssidValid ") + bssidValid);

  // https://randomnerdtutorials.com/esp32-set-custom-hostname-arduino/
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname("ESP32_Fridge");

  // TODO: Ignore bssid for now because IP is flapping
  bssidValid = false;
  logUDP(String("WIFI: IGNORING BSSID ") + wifi_bssid);

  if (bssidValid) {
    logUDP(String("WIFI: CONNECTING WITH BSSID ") + wifi_ssid + " / " + wifi_bssid);
    const int32_t channel = 0;
    const bool connect = true;
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str(), channel, bssid, connect);
  } else {
    logUDP(String("WIFI: CONNECTING ") + wifi_ssid );
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
  }

  digitalWrite(PIN_BUZZER, HIGH);
  for (int retries = 0; WiFi.status() != WL_CONNECTED && retries < 10*WIFI_SEARCH_TIME; retries++) {
    if (verbose) {
      setLed(  0); beep(10);
      setLed(255); delay(90);
    } else {
      beep(5);
      delay(100);
    }

    const int interval = retries % (2*NUMPIXELS); // 0, ..., 23
    const int index = (interval<NUMPIXELS) ? interval : 2*NUMPIXELS-1-interval;
    pixels.clear();
    pixels.setPixelColor(constrain(index, 0, NUMPIXELS-1), pixels.Color(0, 0, 255));
    pixels.show();
  }
  setLed(0);

  if (verbose) {
    if (WiFi.status() != WL_CONNECTED) {
      animateFailure();
    } else {
      animateSuccess();
    }
  }

  if (restartApOnReconnect) {
    const bool wifiOff = false;
    WiFi.softAPdisconnect(wifiOff);
    softApStarted = false;
  }

  if (!softApStarted) {
    String name = "🥶ESP_Fridge_AP🥶";
    if (isDevel()) {
      name += " [devel]";
    }
    WiFi.softAP(name.c_str(), WIFI_AP_PASS);
    IPAddress IP = WiFi.softAPIP();
    logUDP(String("WIFI: NEW AP IP ") + IP.toString());
    softApStarted = true;
  } else {
    IPAddress IP = WiFi.softAPIP();
    logUDP(String("WIFI: KEEPING AP  ") + IP.toString());
  }

  lastWifiSetup = mymillis();
}

#endif