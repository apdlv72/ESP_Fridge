#ifndef serial_h
#define serial_h

void handleSerialCommands() {
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
      // int door = digitalRead(PIN_DOOR);
      // bool isClosed = door == CLOSED;
      // bool isOpen = !isClosed;
      // if (isOpen) {
      //   logUDP("ERROR: CMD DENIED DOOR IS OPEN");
      //   logUDP("ERROR: CMD DENIED DOOR IS OPEN");
      //   logUDP("ERROR: CMD DENIED DOOR IS OPEN");
      // }
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

#endif