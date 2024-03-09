#ifndef ota_h
#define ota_h

volatile bool otaOngoing = false;
int lastOtaPercent = -1;

void handleOTAStart() {
  logUDP("OTA: START");
  logUDP("ACTION: UDP OFF");
  loggingUDP.stop();
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

void setupOta() {
  logUDP("INIT_OTA: START");
  ArduinoOTA.onStart(handleOTAStart);
  ArduinoOTA.onProgress(handleOTAProgress);
  ArduinoOTA.onEnd(handleOTAEnd);

  String name = "ESP_Fridge_OTA";
  bool devel = isDevel();
  if (devel) {
    name += "[devel]";
  }
  logUDP(String("INIT_OTA: NAME ") + name);

  ArduinoOTA.setHostname(name.c_str());
  //ArduinoOTA.setPassword("4711");
  ArduinoOTA.begin();
  logUDP("INIT_OTA: DONE");
}

#endif
