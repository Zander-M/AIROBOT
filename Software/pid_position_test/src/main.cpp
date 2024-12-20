1include <Arduino.h>
#include "pinout.h"

// FastLED
#include <FastLED.h>
#define NUM_LEDS 2

// Elegant OTA 
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>

// Wheels
#include "Wheels.h"


const char* ssid = "test_wifi";
const char* password = "test_password";
CRGB leds[NUM_LEDS];

WebServer server(80);

// put function declarations here:

const byte numChars = 32;
boolean newData = false;
char dataString[numChars];

// OTA callbacks
unsigned long ota_progress_millis = 0;

// Function declaration

// Serial parse-in
void recvWithEndMarker();

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void setup() {

  Serial.begin(115200);

  // Wheels setup
  wheel_setup();


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(5000);

  server.on("/", []() {
    server.send(200, "text/plain", "Hi! This is ElegantOTA Demo.");
  });
  ElegantOTA.begin(&server);    // Start ElegantOTA
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");

  FastLED.addLeds<WS2812, LEDS, RGB>(leds, NUM_LEDS);  // GRB ordering is assumed
}

void loop() {
  recvWithEndMarker();
  if (newData) {
    if (dataString[0] == 'l'){
      setLeftTarget(0);
      setLeftTarget(atoi(&dataString[1]));
    } else {
      setRightTarget(0);
      setRightTarget(atoi(&dataString[1]));
    }
    newData = false;
  }
  wheel_run();
  server.handleClient();
  ElegantOTA.loop();

}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            dataString[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            dataString[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

