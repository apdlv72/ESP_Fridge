#ifndef sensors_h
#define sensors_h

// My productive sensors in descending address order:
//lower: 28d75181e3693c87
//upper: 28adce81e3f33c2c
//rear:  2843cc81e3193c07

// function to format a device address
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
  //logUDP(String("SENSORS: PARASITE ") + (sensors.isParasitePowerMode() ? "ON" : "OFF"));

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
  logUDP(String("SENSORS: 0: ADDR: ") + formatAddress(addresses[0])); // + " PREC: " + sensors.getResolution(addresses[0]));
  logUDP(String("SENSORS: 1: ADDR: ") + formatAddress(addresses[1])); // + " PREC: " + sensors.getResolution(addresses[1]));
  logUDP(String("SENSORS: 2: ADDR: ") + formatAddress(addresses[2])); // + " PREC: " + sensors.getResolution(addresses[2]));
}

#endif
