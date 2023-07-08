
// #include "Adafruit_VL53L0X.h"
// //#include "Wire.h"
// //#include <MPU6050_light.h>
// #include <cmath>
// #include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>

// #define LED_PIN 18
// Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// void setup()
// {
//     //Wire.begin();
//     // pinMode(LED_PIN, OUTPUT);
//     Serial.begin(115200);

//     // wait until serial port opens for native USB devices
//     while (!Serial)
//     {
//         delay(1);
//     }

//     Serial.println("Adafruit VL53L0X test");
//     if (!lox.begin())
//     {
//         Serial.println(F("Failed to boot VL53L0X"));
//         while (1)
//             ;
//     }
//     // power

//     lox.startRangeContinuous();
// }

// void loop()
// {
//     VL53L0X_RangingMeasurementData_t measure;

//     // Serial.print("Reading a measurement... ");
//     // lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

//     if (lox.isRangeComplete())
//     {
//         Serial.print("Distance in mm: ");
//         Serial.println(lox.readRange());
//     }

//     // delay(1);
// }
