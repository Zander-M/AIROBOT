#include <Arduino.h>
#include "pinout.h"

// FastLED
#include <FastLED.h>
#define NUM_LEDS 2

// Mesh
#include <painlessMesh.h>

#define   BLINK_PERIOD    3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for

#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

void sendMessage() ; // Prototype
Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval

// Task to blink the number of nodes
Task blinkNoNodes;
bool onFlag = false;

// Elegant OTA 
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

const char* ssid = "test_wifi";
const char* password = "test_password";
CRGB leds[NUM_LEDS];

WebServer server(80);

// put function declarations here:
const byte numChars = 32;
boolean newData = false;
char dataString[numChars];

// Function declaration
void sendMessage(); 
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

// Serial parse-in
void setup() {

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
  delay(5000);

  // Mesh setup

  mesh.setDebugMsgTypes(ERROR | DEBUG);  // set before init() so that you can see error messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);
  mesh.initOTAReceive("otareceiver");

  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();

  blinkNoNodes.set(BLINK_PERIOD, (mesh.getNodeList().size() + 1) * 2, []() {
      // If on, switch off, else switch on
      if (onFlag)
        onFlag = false;
      else
        onFlag = true;
      blinkNoNodes.delay(BLINK_DURATION);

      if (blinkNoNodes.isLastIteration()) {
        // Finished blinking. Reset task for next run 
        // blink number of nodes (including this node) times
        blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
        // Calculate delay based on current mesh time and BLINK_PERIOD
        // This results in blinks between nodes being synced
        blinkNoNodes.enableDelayed(BLINK_PERIOD - 
            (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
      }
  });
  userScheduler.addTask(blinkNoNodes);
  blinkNoNodes.enable();

  randomSeed(analogRead(A0));
  server.on("/", []() {
    server.send(200, "text/plain", "Hi! This is ElegantOTA Demo.");
  });
  server.begin();
  Serial.println("HTTP server started");

  FastLED.addLeds<WS2812, LEDS, RGB>(leds, NUM_LEDS);  // GRB ordering is assumed
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

void loop() {
  if (onFlag) {
    leds[0] = CRGB::Blue;
    FastLED.show();
  } else {
    leds[0] = CRGB::Green;
    FastLED.show();
  }
  mesh.update();
  server.handleClient();
}

void sendMessage() {
  String msg = "Hello from node ";
  msg += mesh.getNodeId();
  msg += " myFreeMemory: " + String(ESP.getFreeHeap());
  mesh.sendBroadcast(msg);

  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  Serial.printf("Sending message: %s\n", msg.c_str());
  
  taskSendMessage.setInterval( random(TASK_SECOND * 1, TASK_SECOND * 5));  // between 1 and 5 seconds
}


void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> startHere: New Connection, %s\n", mesh.subConnectionJson(true).c_str());
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}

//************************************************************
// this is an example that uses the painlessmesh library to
// upload firmware to another node on the network. 
// This will upload to an ESP device running the otaReceiver.ino
//
// The naming convetions should be as follows for bin files
// firmware_<hardware>_<role>.bin
// To create your own binary files, export them and rename
// them to follow this format, where hardware will be
// "ESP8266" or "ESP32" (capitalized.)
// If sending to the otacreceiver sketch, role should be
// "otareceiver" (lowercase)
//
// This sketch assumes a Nodemcu-32s with an SD card
// connected as shown in the jpg included in the sketch folder
// This code may have to be reworked/hardware adjusted for
// other boards or ESP8266. The core code works well though
//
// MAKE SURE YOUR UPLOADED OTA FIRMWARE INCLUDES OTA SUPPORT 
// OR YOU WILL LOSE THE ABILITY TO UPLOAD MORE FIRMWARE.
// THE INCLUDED .bin DOES NOT HAVE OTA SUPPORT SO THIS MUST BE
// REFLASHED
//************************************************************
#include "painlessMesh.h"

#include <FS.h>
#include "SD.h"
#include "SPI.h"

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

#define OTA_PART_SIZE 1024 //How many bytes to send per OTA data packet

painlessMesh mesh;

void setup() {
  Serial.begin(115200);
  delay(1000);
  mesh.setDebugMsgTypes(
      ERROR | STARTUP | CONNECTION |
      DEBUG);  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);

  // Bridge node, should (in most cases) be a root node. See [the
  // wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation)
  // for some background
  mesh.setRoot(true);
  // This node and all other nodes should ideally know the mesh contains a root,
  // so call this on all nodes
  mesh.setContainsRoot(true);

  delay(1000);


  if (!SD.begin()) {
    rebootEspWithReason("Could not mount SD card, restarting");
  }

  File dir = SD.open("/");
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) { //End of files
      rebootEspWithReason("Could not find valid firmware, please validate");
    }

    //This block of code parses the file name to make sure it is valid.
    //It will also get the role and hardware the firmware is targeted at.
    if (!entry.isDirectory()) {
      TSTRING name = entry.name();
      if (name.length() > 1 && name.indexOf('_') != -1 &&
          name.indexOf('_') != name.lastIndexOf('_') &&
          name.indexOf('.') != -1) {
        TSTRING firmware = name.substring(1, name.indexOf('_'));
        TSTRING hardware =
            name.substring(name.indexOf('_') + 1, name.lastIndexOf('_'));
        TSTRING role =
            name.substring(name.lastIndexOf('_') + 1, name.indexOf('.'));
        TSTRING extension =
            name.substring(name.indexOf('.') + 1, name.length());
        if (firmware.equals("firmware") &&
            (hardware.equals("ESP8266") || hardware.equals("ESP32")) &&
            extension.equals("bin")) {

          Serial.println("OTA FIRMWARE FOUND, NOW BROADCASTING");

          //This is the important bit for OTA, up to now was just getting the file. 
          //If you are using some other way to upload firmware, possibly from 
          //mqtt or something, this is what needs to be changed.
          //This function could also be changed to support OTA of multiple files
          //at the same time, potentially through using the pkg.md5 as a key in
          //a map to determine which to send
          mesh.initOTASend(
              [&entry](painlessmesh::plugin::ota::DataRequest pkg,
                       char* buffer) {
                
                //fill the buffer with the requested data packet from the node.
                entry.seek(OTA_PART_SIZE * pkg.partNo);
                entry.readBytes(buffer, OTA_PART_SIZE);
                
                //The buffer can hold OTA_PART_SIZE bytes, but the last packet may
                //not be that long. Return the actual size of the packet.
                return min((unsigned)OTA_PART_SIZE,
                           entry.size() - (OTA_PART_SIZE * pkg.partNo));
              },
              OTA_PART_SIZE);

          //Calculate the MD5 hash of the firmware we are trying to send. This will be used
          //to validate the firmware as well as tell if a node needs this firmware.
          MD5Builder md5;
          md5.begin();
          md5.addStream(entry, entry.size());
          md5.calculate(); 

          //Make it known to the network that there is OTA firmware available.
          //This will send a message every minute for an hour letting nodes know
          //that firmware is available.
          //This returns a task that allows you to do things on disable or more,
          //like closing your files or whatever.
          mesh.offerOTA(role, hardware, md5.toString(),
                        ceil(((float)entry.size()) / OTA_PART_SIZE), false);

          while (true) {
            //This program will not reach loop() so we dont have to worry about
            //file scope.
            mesh.update();
            usleep(1000);
          }
        }
      }
    }
  }
}

void rebootEspWithReason(String reason) {
  Serial.println(reason);
  delay(1000);
  ESP.restart();
}

void loop(){};