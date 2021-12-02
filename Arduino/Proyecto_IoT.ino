#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <ArduinoJson.h>
#include <ESP8266httpUpdate.h>

//Datos para actualización:
#define HTTP_OTA_ADDRESS      F(Wifi.localIP().toString().c_str())    // Address of OTA update server
#define HTTP_OTA_PATH         F("/esp8266-ota/update")                // Path to update firmware
#define HTTP_OTA_PORT         1880                                    // Port of update server                                                          
#define HTTP_OTA_VERSION      String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".nodemcu"  // Name of firmware

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
