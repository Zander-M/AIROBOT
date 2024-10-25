#include <Arduino.h>
#include "pinout.h"

// Elegant OTA stuff
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>

const char* ssid = "test_wifi";
const char* password = "test_password";

WebServer server(80);

// put function declarations here:

int left_counter = 0;
int right_counter = 0;
char s[100];
int speed = 255;
int i = 0;

// OTA callbacks
unsigned long ota_progress_millis = 0;

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

// Encoder interrupts
void left_counter_update(){
  if (digitalRead(E2B) == LOW){
    left_counter ++;
  } else {
    left_counter --;
  }
}

void right_counter_update(){
  if (digitalRead(E1B) == LOW){
    right_counter ++;
  } else {
    right_counter --;
  }
}
void setup() {

  // Encoders
  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2A, INPUT);

  // Motor
  pinMode(ME1A, OUTPUT);
  pinMode(ME1B, OUTPUT);
  pinMode(ME2A, OUTPUT);
  pinMode(ME2B, OUTPUT);

  // LEDs
  pinMode(LEDS, OUTPUT);

  attachInterrupt(E1A, left_counter_update, RISING);
  attachInterrupt(E2A, right_counter_update, RISING);
  Serial.begin(115200);

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

  server.on("/", []() {
    server.send(200, "text/plain", "Hi! This is ElegantOTA Demo.");
  });
  ElegantOTA.begin(&server);    // Start ElegantOTA
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");
}

// Forward
void forward(int distance, int speed, int decay) {
  // Soft start/stop
  digitalWrite(ME1A, HIGH);
  digitalWrite(ME2A, LOW);
  for (int i=150; i < speed; ++i){
    analogWrite(ME1B, 255-i);
    analogWrite(ME2B, i);
    delay(decay);
  }
  delay(10000); // TODO: calculate distance based on encoder readings

  for (int i=speed; i > 150; --i){
      analogWrite(ME1B, 255-i);
      analogWrite(ME2B, i);
      delay(decay);
    }
  analogWrite(ME1B, 255);
  analogWrite(ME2B, 0);
  delay(10);
}

// Backward
void backward(int distance, int speed, int decay) {
  // Soft start/stop
  digitalWrite(ME1A, LOW);
  digitalWrite(ME2A, HIGH);
  for (int i=150; i < speed; ++i){
    analogWrite(ME1B, i);
    analogWrite(ME2B, 255-i);
    delay(decay);
  }
  delay(10000); // TODO: calculate distance based on encoder readings

  for (int i=speed; i > 150; --i){
      analogWrite(ME1B, i);
      analogWrite(ME2B, 255-i);
      delay(decay);
    }
  analogWrite(ME1B, 0);
  analogWrite(ME2B, 255);
  delay(10);

}


void loop() {
  // put your main code here, to run repeatedly:
  while (i < 3){ 
    forward(10, 200, 5);
    delay(500);
    backward(10, 200, 5);
    delay(500);
    ++i;
  }

  server.handleClient();
  ElegantOTA.loop();

  // delay(1000);
  // digitalWrite(ME1A, LOW);
  // digitalWrite(ME1B, LOW);
  // delay(10);
  // sprintf(s, "%d, %d", left_counter, right_counter);
  // Serial.println(s);
  // delay(50);

  // digitalWrite(ME1A, HIGH);
  // analogWrite(ME1B, 255-speed);
  // delay(1000);

  // digitalWrite(ME1A, LOW);
  // digitalWrite(ME1B, LOW);

  // delay(10);
  // sprintf(s, "%d, %d", left_counter, right_counter);
  // Serial.println(s);
  // delay(50);

}

