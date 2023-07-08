// /* Get tilt angles on X and Y, and rotation angle on Z
//     Angles are given in degrees
//  License: MIT
//  */

// #include "Wire.h"
// #include <MPU6050_light.h>
// #include <cmath>
// #include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>

// #define LED_PIN 2

// MPU6050 mpu(Wire);
// unsigned long timer = 0;
// void setup()
// {
//     Serial.begin(115200);
//     Wire.begin();
//     pinMode(LED_PIN, OUTPUT);

//     byte status = mpu.begin();
//     Serial.print(F("MPU6050 status: "));
//     Serial.println(status);
//     while (status != 0)
//     {
//     } // stop everything if could not connect to MPU6050
//     Serial.println(F("Calculating offsets, do not move MPU6050"));
//     delay(1000);
//     mpu.calcOffsets(); // gyro and accelero
//     Serial.println("Done!\n");
// }
// void loop()
// {
//     mpu.update();

//     double centri_acc = sqrt(pow(mpu.getAccX(), 2) + pow(mpu.getAccY(), 2));
//     Serial.print("centri_acc : ");
//     Serial.println(centri_acc);
   
//     // Serial.println(millis());
//     if ((millis() - timer) > 10)
//     { // print data every 10ms
//         // Serial.print("X : ");
//         // Serial.print(mpu.getAngleX());
//         // Serial.print("\tY : ");
//         // Serial.print(mpu.getAngleY());
//         // Serial.print("\tZ : ");
//         // Serial.println(fmod(mpu.getAngleZ(), 360));
//         // timer = millis();
//     }
// }
