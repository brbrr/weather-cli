#include <Arduino.h>

#include <Hacked_ESP8266.h>
#include <SoftwareSerial.h>
#include "DHT.h" //DHT
#include <Wire.h> //BMP
#include <OneWire.h> //DS1820
#include <DallasTemperature.h> //DS1820
#include <BMP180.h> //BMP
#include <ArduinoJson.h>


void doRequest(Fstr *url, char* data);
void initSensors(void);
float getHumidity(void);
float getTemperature(void);
void clearBuffer(void);
char* createJson(char* buff, int len);

#define ARD_RX   10
#define ARD_TX   11
#define ESP_RST   5

#define ESP_SSID "Mystical Forcess" // Your network name here
#define ESP_PASS "roomofmagic!" // Your network password here

#define HOST     "192.168.0.50"     // Host to contact
#define PAGE     "/reports" // Web page to request
#define PORT     4567        // 80 = HTTP default port

//#define esp Serial1 // If using Leonardo board
#define debug Serial // Ideally switch to external (SS debug UART), as now debug output contains lots of trash

SoftwareSerial esp(ARD_RX, ARD_TX); // Arduino RX = ESP TX, Arduino TX = ESP RX

// Must declare output stream before Adafruit_ESP8266 constructor; can be
// a Softwareserial stream, or debug/debug1/etc. for UART.
// Must call begin() on the stream(s) before using Adafruit_ESP8266 object.
Hacked_ESP8266 wifi(&esp, &debug, ESP_RST);

// Sensor classes
DHT dht(3, DHT11); // DHT sensor on pin 3
BMP180 barometer;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(4); //DS1820 sensor on pin 4
DallasTemperature sensors(&oneWire);
DeviceAddress ds; // arrays to hold device address

unsigned long previousMillis = 0;        // will store last time request was send

void setup() {
        esp.begin(9600); // Soft debug connection to ESP8266
        debug.begin(9600);
        //while (!debug); // UART debug, Actual for Leonardo board

        debug.println(F("Starting up the weather-cli"));
        debug.println(F("Sensors init"));
        initSensors();

        wifi.setBootMarker(F("ready"));
        // wifi.setTimeouts(20000, 20000, 30000, 20000);

        // Test if module is ready
        debug.print(F("Hard reset..."));
        if (!wifi.hardReset()) {
                debug.println(F("no response from module."));
                for (;; );
        }
        debug.println(F("OK."));

        debug.print(F("Soft reset..."));
        if (!wifi.softReset()) {
                debug.println(F("no response from module."));
                for (;; );
        }
        debug.println(F("OK."));

        debug.print(F("Checking firmware version..."));
        wifi.println(F("AT+GMR"));
        char buffer[50];
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
                        char b[200];
                        doRequest(F(PAGE), createJson(b, 200));
                } else debug.println(F("error")); // IP addr check failed
                //wifi.closeAP();
        } else { // WiFi connection failed
                debug.println(F("FAIL"));
        }
}

void loop() {
        char buff[128];
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= 120000L) {
                previousMillis = currentMillis;
                createJson(buff, 128);
                debug.println(buff);
                doRequest(F(PAGE), buff);
        }
}

void doRequest(Fstr *url, char* data) {
        debug.print(F("Connecting to host..."));
        if (wifi.connectTCP(F(HOST), PORT)) {
                debug.print(F("OK\nRequesting page..."));
                if (wifi.postRequest(url, data)) {
                        debug.println("OK\nSearching for string...");
                        // Search for a phrase in the open stream.
                        // Must be a flash-resident string (F()).
                        if (wifi.find(F("201 Created"), true)) {
                                debug.println(F("found!"));
                                clearBuffer();
                        } else {
                                debug.println(F("not found."));
                        }
                } else { // URL request failed
                        debug.println(F("error"));
                }
                debug.print(F("Closing connection.."));
                wifi.closeTCP();
        } else { // TCP connect failed
                debug.println(F("D'oh!"));
        }
        debug.println(F("EXIT"));
}

// Initating BMP sensor
void initSensors(void) {
        sensors.begin(); // DS1820
        dht.begin(); // DHT
        barometer = BMP180(); // BMP
        if (barometer.EnsureConnected()) { // We check to see if we can connect to the sensor.
                debug.println(F("Connected to BMP180")); // Output we are connected to the computer.
                barometer.SoftReset(); // When we have connected, we reset the device to ensure a clean start.
                barometer.Initialize(); // Now we initialize the sensor and pull the calibration data.
        }
        else debug.println(F("Could not connect to BMP180"));
        if (!sensors.getAddress(ds, 0)) debug.println("Unable to find address for Device 0");
        sensors.setResolution(ds, 9);
}

// Humidity from DHT
float getHumidity(void) {
        float h = dht.readHumidity();
        if (isnan(h)) { // Check if any reads failed and exit early (to try again).
                debug.println(F("Failed to read from DHT sensor!"));
                h = -99;
        }
        return h;
}

// Temp from DS1820
float getTemperature(void) {
        sensors.requestTemperatures(); // Send the command to get temperatures
        float tempC = sensors.getTempC(ds);
        return tempC;
}

// Cleaning up the esp buffer
// useful when finding something in middle of the buffer.
// preventing logging of remain buffer part
void clearBuffer(void) {
        while (esp.available() > 0) {
                esp.read();
                delay(1);
        }
}

char* createJson(char* buff, int len) {
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        root["temperature"] = getTemperature();
        root["humidity"] = getHumidity();
        root["preasure"] = barometer.GetPressure();
        root.printTo(buff, len);
        return(buff);
}
