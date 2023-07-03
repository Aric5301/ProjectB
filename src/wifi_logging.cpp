// /*********
//   Rui Santos
//   Complete project details at https://RandomNerdTutorials.com
  
//   Permission is hereby granted, free of charge, to any person obtaining a copy
//   of this software and associated documentation files.
  
//   The above copyright notice and this permission notice shall be included in all
//   copies or substantial portions of the Software.
// *********/

// // Import required libraries

// #include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>
// #include <Wire.h>

// /*#include <SPI.h>
// #define BME_SCK 18
// #define BME_MISO 19
// #define BME_MOSI 23
// #define BME_CS 5*/

// //Adafruit_BME280 bme(BME_CS); // hardware SPI
// //Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// // Replace with your network credentials
// const char* ssid = "Ziv's Phone";
// const char* password = "psagot22";

// // Create AsyncWebServer object on port 80
// AsyncWebServer server(80);

// String readBME280Temperature() {
//   return String(1);
// }

// String readBME280Humidity() {
//    return String(2);
// }

// String readBME280Pressure() {
//    return String(3);
// }

// void setup(){
//   // Serial port for debugging purposes
//   Serial.begin(115200);


//   // Initialize SPIFFS
//   if(!SPIFFS.begin()){
//     Serial.println("An Error has occurred while mounting SPIFFS");
//     return;
//   }

//   // Connect to Wi-Fi
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(1000);
//     Serial.println("Connecting to WiFi..");
//   }

//   // Print ESP32 Local IP Address
//   Serial.println(WiFi.localIP());

//   // Route for root / web page
//   server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//     request->send(SPIFFS, "/index.html");
//   });
//   server.on("/gyro", HTTP_GET, [](AsyncWebServerRequest *request){
//     request->send_P(200, "text/plain", readBME280Temperature().c_str());
//   });
//   server.on("/tfluna", HTTP_GET, [](AsyncWebServerRequest *request){
//     request->send_P(200, "text/plain", readBME280Humidity().c_str());
//   });
//   server.on("/vl53", HTTP_GET, [](AsyncWebServerRequest *request){
//     request->send_P(200, "text/plain", readBME280Pressure().c_str());
//   });

//   // Start server
//   server.begin();
// }
 
// void loop(){
  
// }