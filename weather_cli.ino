#include <Adafruit_ESP8266.h>
#include <SoftwareSerial.h>
#include <DHT.h> // DHT
#include <OneWire.h> // DS1820
#include <Wire.h>
#include <BMP180.h> // BMP

#define ARD_RX_ESP_TX   10
#define ARD_TX_ESP_RX   11
#define ESP_RST         5
SoftwareSerial esp(ARD_RX_ESP_TX, ARD_TX_ESP_RX); // Arduino RX = ESP TX, Arduino TX = ESP RX
//SoftwareSerial debug(ARD_RX_ESP_TX, ARD_TX_ESP_RX); // Arduino RX = ESP TX, Arduino TX = ESP RX

#define debug Serial
//#define esp Serial1
// Must declare output stream before Adafruit_ESP8266 constructor; can be
// a Softwaredebug stream, or debug/debug1/etc. for UART.
Adafruit_ESP8266 wifi(&esp, &debug, ESP_RST);  // Must call begin() on the stream(s) before using Adafruit_ESP8266 object.

DHT dht(3, DHT11); // DHT sensor on pin 2
OneWire  ds(4);  // on pin 10 (a 4.7K resistor is necessary)
BMP180 barometer;

#define ESP_SSID "Mystical Forcess" // Your network name here
#define ESP_PASS "roomofmagic!" // Your network password here

#define HOST     "192.168.0.50"     // Host to contact
#define PAGE     "/reports" // Web page to request
#define PORT     4567        // 80 = HTTP default port

unsigned long previousMillis = 0;        // will store last time request was send


void setup() {
  char buffer[50];

  esp.begin(9600); // Soft debug connection to ESP8266
  debug.begin(9600);
  //while (!debug); // UART debug

  debug.println(F("Adafruit ESP8266 Demo"));

  barInit();

  wifi.setBootMarker(F("ready"));
  wifi.setTimeouts(20000, 20000, 30000, 20000);

  // Test if module is ready
  debug.print(F("Hard reset..."));

  if (!hardReset()) {
    debug.println(F("no response from module."));
    for (;;);
  }

  debug.println(F("OK."));

  debug.print(F("Soft reset..."));
  if (!softReset()) {
    debug.println(F("no response from module."));
    for (;;);
  }
  debug.println(F("OK."));

  debug.print(F("Checking firmware version..."));
  wifi.println(F("AT+GMR"));
  if (wifi.readLine(buffer, sizeof(buffer))) {
    debug.println(buffer);
    wifi.find(F("OK.")); // Discard the 'OK' that follows
  } else {
    debug.println(F("error"));
  }

  debug.print(F("Connecting to WiFi..."));
  if (wifi.connectToAP(F(ESP_SSID), F(ESP_PASS))) {
    debug.print(F("OK\nChecking IP addr..."));
    wifi.println(F("AT+CIFSR"));
    if (wifi.readLine(buffer, sizeof(buffer))) {
      debug.println(buffer);
      wifi.find(); // Discard the 'OK' that follows
      doRequest();
    } else { // IP addr check failed
      debug.println(F("error"));
    }
    //wifi.closeAP();
  } else { // WiFi connection failed
    debug.println(F("FAIL"));
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 120000L) {
    previousMillis = currentMillis;
    doRequest();
  }
}

void doRequest() {
  debug.print(F("Connecting to host..."));
  if (wifi.connectTCP(F(HOST), PORT)) {
    debug.print(F("OK\nRequesting page..."));
    if (wifi.postRequest(F(PAGE), prepareJson())) {
      debug.println("OK\nSearching for string...");
      // Search for a phrase in the open stream.
      // Must be a flash-resident string (F()).
      if (wifi.find(F("created_at"), true)) {
        debug.println(F("found!"));
      } else {
        debug.println(F("not found."));
      }
    } else { // URL request failed
      debug.println(F("error"));
    }
    wifi.closeTCP();
  } else { // TCP connect failed
    debug.println(F("D'oh!"));
  }
}

bool hardReset() {
  if (ESP_RST < 0) return true;
  digitalWrite(ESP_RST, LOW);
  pinMode(ESP_RST, OUTPUT); // Open drain; reset -> GND
  delay(100);                  // Hold a moment
  pinMode(ESP_RST, INPUT);
  digitalWrite(ESP_RST, HIGH); // Back to high-impedance pin state
  return wifi.find(F("ready"));    // Purge boot message from stream
}

bool softReset(void) {
  boolean  found = false;
  //uint32_t save  = wifi.receiveTimeout; // Temporarily override recveive timeout,
  // wifi.receiveTimeout = wifi.resetTimeout;   // reset time is longer than normal I/O.
  esp.println(F("AT+RST"));            // Issue soft-reset command
  if (wifi.find(F("ready"))) {          // Wait for boot message
    esp.println(F("ATE0"));            // Turn off echo
    found = wifi.find();                // OK?
  }
  //wifi.receiveTimeout = save;           // Restore normal receive timeout
  return found;
}

String prepareJson() {
  String result = "{\"temperature\":";
  result += String(getTemperature());
  result += ",\"humidity\":";
  result += String(getHumidity());
  result += ",\"preasure\":";
  result += String(barometer.GetPressure());
  result += "}\r\n";
  return result;
}

// Initating BMP sensor
void barInit(void) {
  //barometer = BMP180();
  if (barometer.EnsureConnected()) { // We check to see if we can connect to the sensor.

    debug.println(F("Connected to BMP180")); // Output we are connected to the computer.
    barometer.SoftReset(); // When we have connected, we reset the device to ensure a clean start.
    barometer.Initialize(); // Now we initialize the sensor and pull the calibration data.
  }
  else {
    debug.println(F("Could not connect to BMP180"));
  }
}

// Humidity from DHT
float getHumidity(void) {
  float h = dht.readHumidity();
  if (isnan(h)) { // Check if any reads failed and exit early (to try again).
    debug.println(F("Failed to read from DHT sensor!"));
  } else {
    return h;
  }
}

// Temp from DS1820
float getTemperature() {
  byte type_s;
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
  }
  switch (addr[0]) {
    case 0x10: // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      debug.println(F("Device is not a DS18x20 family device."));
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  delay(750);     // maybe 750ms is enough, maybe not
  ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad
  for ( int i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6]; // "count remain" gives full 12 bit resolution
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  return (float)raw / 16.0;
}
