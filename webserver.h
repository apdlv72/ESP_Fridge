#ifndef webserver_h
#define webserver_h

const char FAVICON_ICO[] PROGMEM =
#include "favicon.ico.h"
;

const char FRIDGE192_PNG[] PROGMEM =
#include "fridge192.png.h"
;

const char MANIFEST_JSON[] PROGMEM =
#include "manifest.json.h"
;

WebServer server(80);

void handleNotFound() {
  logUDP("DEBUG: handleNotFound");
  server.send(404, "text/plain", "File Not Found\n\n");
}


String toHumanReadableTime(int secs) {
  long mins = secs / 60; secs %= 60;
  long hrs  = mins / 60; mins %= 60;
  long days = hrs  / 24; hrs  %= 24;
  String t;
  if (days > 0) {
    t += String(days) + "d";
  }
  if (t.length() > 0 || hrs > 0) {
    if (t.length() > 0) t += ", ";
    t += String(hrs ) + "h";
  }
  if (t.length() > 0 || mins > 0) {
    if (t.length() > 0) t += ", ";
    t += String(mins) + "m";
  }
  if (t.length() > 0) t += ", ";
  t += String(secs) + "s";
  return t;
}

void serverIndex() {
  logUDP("DEBUG: serverIndex");
  const unsigned long uptime = mymillis();
  const unsigned long motorOnDuration = uptime - lastMotorOnTime;

  const int doorStatus = digitalRead(PIN_DOOR);
  const bool isDoorOpen = (doorStatus != CLOSED);
  const unsigned long doorOpenDuration = uptime - lastClosed;

  String commit = String("$Id: 6fa90f5baa18340168c10ff03381665158f8e355 $");
  String hash = commit.substring(5).substring(0, 7);
  String addr = WiFi.localIP().toString();

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
  html += "<center><img src=\"fridge192.png\" border=\"0\"></center>\n";
  html += "<form action=\"/set\">\n";
  html += "<table>\n";  
  //timeClient.getFormattedDate();
  html += "<tr><td>Time:</td><td>" + timeClient.getFormattedTime() + "</td></tr>\n";
  //html += "<tr><td>Day:</td><td>" + String(timeClient.getDay()) + "</td></tr>\n";
  html += String("<tr><td>Start time:</td><td>") + startTime + "</td></tr>\n";
  html += "<tr><td>Build:</td><td>" __DATE__ " " __TIME__ " </td></tr>\n";
  html += "<tr><td>Commit:</td><td>" + hash + "</td></tr>\n";
  html += "<tr><td>Reboots:</td><td>" + String(reboots) + "</td></tr>\n";
  html += String("<tr><td>Address:</td><td><a style=\"text-decoration:none\" target=\"fridge\" href=\"http://") + addr + String("\">") + addr + "</a></td></tr>\n";
  html += "<tr><td>Uptime:</td><td>" + toHumanReadableTime(uptime/1000) + "</td></tr>\n";
  html += "<tr><td>Upper temp:</td><td>" + String(upperC) + " &deg;C</td></tr>\n";
  html += "<tr><td>Lower temp:</td><td>" + String(lowerC) + " &deg;C</td></tr>\n";
  html += "<tr><td>Rear  temp:</td><td>" + String(rearC)  + " &deg;C</td></tr>\n";

  html += "<tr><td>Compressor:</td><td>";
  if (comprActive) {
    unsigned duration = uptime-lastMotorOnTime;
    html += String("ACTIVE ") + duration; 
  } else {
    unsigned long motorOffDuration = uptime - lastMotorOffTime;
    html += String(comprRequired ? "REQUIRED " : "OFF ") + motorOffDuration + "/" + String(MOTOR_MIN_OFF_TIME);
  } 
  html += "</td></tr>\n";
  
  html += "<tr><td>Door:</td><td>";
  if (isDoorOpen) {
      html += "OPEN ";
      html += doorOpenDuration;
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
  html += "<center>\n";
  html += "<a href=\"/wifi.html\">[WIFI]</a>\n";
  html += "<a href=\"/test.html\">[TEST]</a>\n";
  html += "</center>\n";
  html += "\n</body>\n</html>\n";
  server.send(200, "text/html", html);
}

void serverRedirect(String msg, String url, unsigned int seconds) {
  logUDP("DEBUG: serverRedirect");
  String html = String("<html>\n");
  html += "<head>\n";
  html += "<meta http-equiv=\"refresh\" content=\"" + String(seconds) + "; URL=" + url + "\">\n";
  html += "</head>\n";
  html += "<body style=\"font-family: verdana, arial, sans-serif; background-color: #a0ffa0; font-size: 48px;\">\n";
  html += "<center><h1><br/><br/><br/>";
  html += msg;
  html += "</h1></center>\n";
  html += "</body>\n";
  html += "</html>\n";
  server.send(200, "text/html", html);
}

void serverTestaction() {

  String confirm = server.arg(0);
  if (confirm!="confirm") {
    serverRedirect("NO CONFIRM", "/test.html", 3);
    return;
  }

  String submit = server.arg(1);
  logUDP(String("TEST: ") + submit);
  beep(30);

  if (submit=="RST") {
    logUDP("DEBUG: serverReset");
    serverRedirect("RESET", "/test.html", 10);
    ESP.restart();
  } else if (submit=="FON") {
    setFan(255);
    serverRedirect("FAN 100%", "/test.html", 3);
  } else if (submit=="FOFF") {
    setFan(0);
    serverRedirect("FAN 0%", "/test.html", 3);
  } else if (submit=="CON") {      
    setCompressor(1, false);
    serverRedirect("COMPRESSOR ON", "/test.html", 3);
  } else if (submit=="COFF") {      
    setCompressor(0, false);
    serverRedirect("COMPRESSOR OFF", "/test.html", 3);
  } else {
    serverRedirect(String("UNKNOWN ACTION ") + submit, "/test.html", 5);
  }
}

void serverSetWifi() {
  String ssid       = server.arg(0);
  String bssidStr   = server.arg(1);
  String password   = server.arg(3);

  logUDP(String("WIFI: SSID ") + ssid);
  logUDP(String("WIFI: BSSID ") + bssidStr);

  uint8_t buff[6];
  const bool bssidValid = parseBssid(bssidStr, buff);

  // ESP32 may have issues when forcing channel
  const int32_t channel = 0; //channelStr.toInt();
  const bool channelValid = channel>=0;

  if (!bssidValid) {
    bssidStr="INVALID";
  }
  if (password=="") {
    serverRedirect(String("WIFI: NO PASSWORD "), "/wifi.html", 3);
    return;
  }

  logUDP(String("WIFI: SSID '") + ssid + "'");
  logUDP(String("WIFI: BSSID '") + bssidStr + "'");

  if (bssidValid && channelValid) {
    const bool connect = true;
    logUDP(String("WIFI: CONNECT WITH BSSID ") + ssid + String(" / ") + bssidStr);
    WiFi.begin(ssid.c_str(), password.c_str(), channel, buff, connect);
  } else {
    logUDP(String("WIFI: CONNECT ") + ssid);
    WiFi.begin(ssid.c_str(), password.c_str());
  }

  for (int retries = 0; WiFi.status() != WL_CONNECTED && retries < 10*WIFI_SEARCH_TIME; retries++) {
    beep(10);
    delay(100);    
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI: FAILED");
    beep(1000);
    serverRedirect(String("FAILED"), "/wifi.html", 3);
  } else {    
    logUDP("WIFI: SUCCEEDED");
    preferences.putString(KEY_WIFI_SSID,  ssid);
    preferences.putString(KEY_WIFI_BSSID, bssidStr);
    preferences.putString(KEY_WIFI_PASSWORD, password);
    logUDP("PREFS: SAVED");
    for (int i = 0; i < 3; i++) {
      setLed(16); digitalWrite(PIN_BUZZER, LOW);
      delay(100);
      setLed(0); digitalWrite(PIN_BUZZER, HIGH);
      delay(100);
    }
    serverRedirect(String("OK"), "/wifi.html", 3);
  }
}

void serverWifiSetup() {
  String html = String("<!DOCTYPE html PUBLIC \"-//WAPFORUM//DTD XHTML Mobile 1.0//EN\" \"http://www.wapforum.org/DTD/xhtml-mobile10.dtd\">\n");
  html += "<html>\n";
  html += "<head>\n";
  html += "<meta charset='utf-8'>\n";
  html += "<title>Fridge WiFi</title>\n";
  html += "<script>\n";
  html += "function copy(elem){\n";
  html += "  var ssid  = elem.getElementsByClassName('ssid')[0];\n";
  html += "  var bssid = elem.getElementsByClassName('bssid')[0];\n";
  html += "  var channel = elem.getElementsByClassName('channel')[0];\n";
  html += "  document.getElementById('ssid').value  = ssid.innerHTML;\n";
  html += "  document.getElementById('bssid').value = bssid.innerHTML;\n";
  html += "  document.getElementById('channel').value = channel.innerHTML;\n";
  html += "}\n";
  html += "</script>\n";
  html += "</head>\n";

  html += "<body>\n";
  html += "<h1>WIFI SETUP</h1>\n";
  html += "<style type=\"text/css\">\n";
  html += "body { font-family: verdana, arial, sans-serif; background-color: #a0a0ff; }\n";
  html += "</style>\n";
  html += "<body>\n";

  logUDP("WIFI: START SCAN");
  int n = WiFi.scanNetworks();
  logUDP("WIFI: SCAN DONE");

  for (int i = 0; i < n; ++i) {
    html += "<p id=\"" + String(i)+ "\"onClick=\"copy(this)\">";
    html += String("  <span class=\"ssid\">" ) + WiFi.SSID(i)     + "</span><br/>\n";
    html += String("  <span class=\"bssid\">") + WiFi.BSSIDstr(i) + "</span><br/>\n";
    html += String("  <span class=\"rssi\">" ) + WiFi.RSSI(i)     + " dBm</span><br/>\n";
    html += String("  Channel <span class=\"channel\">") + WiFi.channel(i) + "</span><br/>\n";
    html += "</p>\n";
  }

  html += "<form action=\"/setwifi\">\n";
  html += "Network:<br/>\n";
  html += "<input id=\"ssid\" type=\"text\" name=\"ssid\"><br/>\n";
  html += "BSSID:<br/>\n";
  html += "<input id=\"bssid\" type=\"text\" name=\"bssid\"><br/>\n";
  html += "Channel:<br/>\n";
  html += "<input id=\"channel\" type=\"hidden\" name=\"channel\"><br/>\n";
  html += "Password:<br/>\n";
  html += "<input type=\"text\" name=\"pass\"><br/>\n";
  html += "<input type=\"submit\" name=\"submit\" value=\"save\"><br/>\n";
  html += "</form>\n";
  html += "<br/><br/>\n";
  html += "<a href=\"/index.html\">[BACK]</a>\n";
  html += "</body>\n";
  html += "</html>\n";
  server.send(200, "text/html", html);
}

void serverTestpage() {
  String html = String("<!DOCTYPE html PUBLIC \"-//WAPFORUM//DTD XHTML Mobile 1.0//EN\" \"http://www.wapforum.org/DTD/xhtml-mobile10.dtd\">\n");
  html += "<html>\n";
  html += "<head>\n";
  html += "<title>Fridge Test</title>\n";
  html += "<style type=\"text/css\">\n";
  html += "body { font-family: verdana, arial, sans-serif; background-color: #a0a0ff; }";
  html += "</style>";
  html += "</head>\n";
  html += "<body>\n";
  html += "<h1>TEST PAGE</h1>\n";
  html += "<form action=\"/testaction\">\n";
  //html += "<center>\n";
  html += "<table>\n";
  html += String("<tr><td>BSSID: </td><td>") + WiFi.BSSIDstr();
  html += "</td></tr>\n";  
  html += String("<tr><td>RSSI: </td><td>") + WiFi.RSSI();
  html += "</td></tr>\n";  
  html += String("<tr><td>T_UPPER_HI: </td><td>") + T_UPPER_HI;
  html += "</td></tr>\n";
  html += String("<tr><td>T_UPPER_LO: </td><td>") + T_UPPER_LO;
  html += "</td></tr>\n";
  html += String("<tr><td>T_LOWER_HI: </td><td>") + T_LOWER_HI;
  html += "</td></tr>\n";
  html += String("<tr><td>T_LOWER_LO: </td><td>") + T_LOWER_LO;
  html += "</td></tr>\n";
  html += "</table>\n";
  //html += "</center>\n";
  html += "<br/><br/>\n";
  html += "confirm: <input type=\"text\" name=\"confirm\"><br/><br/>\n";
  html += "<input type=\"submit\" name=\"RST\"  value=\"RST\">\n";
  html += "<input type=\"submit\" name=\"CON\"  value=\"FON\">\n";
  html += "<input type=\"submit\" name=\"COFF\" value=\"FOFF\">\n";
  html += "<input type=\"submit\" name=\"FON\"  value=\"CON\">\n";
  html += "<input type=\"submit\" name=\"FOFF\" value=\"COFF\">\n";
  html += "</form>\n";
  html += "</center><br/><br/>\n";
  html += "<center><a href=\"/index.html\">[BACK]</a></center>\n";
  html += "</body>\n";
  html += "</html>\n";
  server.send(200, "text/html", html);
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
  serverRedirect("OK", "/", 2);
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
  server.on("/", serverIndex);
  server.on("/index.html", serverIndex);
  server.on("/set", serverSet);

  server.on("/wifi.html", serverWifiSetup);
  server.on("/setwifi",   serverSetWifi);

  server.on("/test.html", serverTestpage);
  server.on("/testaction", serverTestaction);

  server.on("/manifest.json", handleManifest);
  server.on("/favicon.ico",   handleFavicon);
  server.on("/fridge192.png", handleHomeIcon192);

  server.onNotFound(handleNotFound);
  server.begin();
}


#endif