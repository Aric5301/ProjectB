#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <cmath>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#include <iostream>
#include <iterator>
#include <map>
#include <queue>
using namespace std;

#define LED_PIN 2
#define RXD2 16
#define TXD2 17
#define MIN_ESTIMATED_ROTATION_FREQ 0 // in Hz
#define MAX_ESTIMATED_ROTATION_FREQ 3 // in Hz
#define ROTATION_FREQ_GRANULARITY 0.2 // in Hz
#define TF_LUNA_FREQ 250              // in Hz
#define SAMPLE_BUFFER_SIZE 20

std::map<int, int> mapOfEnvironment; // maps half degrees to distances. key=0 is angle 0, key=1 is angle 0.5, key=2 is angle 1, ... key=719 is 359.5.
queue<int> samplesBuffer;

double currentEstimatedAngle;

// WiFi credentials
const char *ssid = "Aric's OnePlus 8T";
const char *password = "16901690";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU6050 mpu = MPU6050(Wire);

double getGyroYaw();
int getLunaDistance();
void updateLunaDistance();
std::vector<double> linspace(double start, double end, int numPoints);
void initSamplesBuffer();
void pushSampleToQueue(int distance);
void addMeasurmentToMapOfEnvironment(double yaw, int distance);
std::vector<int> sampleMapClosestKey(const std::map<int, int> &inputMap, const std::vector<int> &keys);
std::vector<int> multiplyVectorByScalarAndCastToInt(const std::vector<double> &vec, double scalar);
std::vector<int> normalizeVectorBy360(const std::vector<int> &vec);
int scalarDifference(vector<int> vec, queue<int> que);
std::vector<int> addScalar(const std::vector<int> &vec, int scalar);
double estimateCurrentAngle();
void printVector(const std::vector<int> &vec);

template <typename Key, typename Value>
void printMapValues(const std::map<Key, Value> &map);

String readGyro()
{
  return String(getGyroYaw());
}

String readTfluna()
{
  return String(getLunaDistance());
}

String readVl53()
{
  return String(currentEstimatedAngle);
}

void setup()
{
  Serial.begin(115200);  // Communication with host
  Serial2.begin(115200); // Communication with TF-Luna
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);

  // wait until serial port opens for native USB devices
  while (!Serial)
  {
    delay(1);
  }

  // Boot MPU6050
  // ==========================================================================================
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  if (status != 0) // stop everything if could not connect to MPU6050
  {
    Serial.println(F("Failed to boot MPU6050."));
    while (1)
      ;
  }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println(F("Done!\n"));
  // ==========================================================================================

  // Initialize SPIFFS
  // ==========================================================================================
  if (!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // ==========================================================================================

  char freq_packet[] = {0x5A, 0x06, 0x03, 0xFA, 0x00, 0x00}; // set TF-Luna freq to 250 Hz (0xFA=250)
  Serial2.write(freq_packet, (size_t)6);

  // Connect to Wi-Fi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi..");
  // }

  // // Print ESP32 Local IP Address
  // Serial.println(WiFi.localIP());

  // // Route for root / web page
  // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send(SPIFFS, "/index.html"); });
  // server.on("/gyro", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send_P(200, "text/plain", readGyro().c_str()); });
  // server.on("/tfluna", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send_P(200, "text/plain", readTfluna().c_str()); });
  // server.on("/vl53", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send_P(200, "text/plain", readVl53().c_str()); });

  // // Start server
  // server.begin();

  initSamplesBuffer();
}

bool theFlag = true;
unsigned long prevMillisOfDistanceSample;
void loop()
{
  mpu.update();
  updateLunaDistance();

  // if (mapOfEnvironment.size() < 360)
  // {
  //   Serial.print("TF-Luna distance(cm) = ");
  //   Serial.println(getLunaDistance());

  //   // Serial.print("Gyro yaw (degrees) = ");
  //   // Serial.println(getGyroYaw());

  //   Serial.print("Current size of mapOfEnvironment = ");
  //   Serial.println(mapOfEnvironment.size());
  //   addMeasurmentToMapOfEnvironment(getGyroYaw(), getLunaDistance());
  // }
  // else if (theFlag)
  // {
  //   printMapValues(mapOfEnvironment);
  //   theFlag = false;
  // }
  // else if ((millis() - prevMillisOfDistanceSample) >= (1000 / TF_LUNA_FREQ))
  // {
  //   pushSampleToQueue(getLunaDistance());
  //   prevMillisOfDistanceSample = millis();
  // }
  // else
  // {
  //   currentEstimatedAngle = estimateCurrentAngle();
  //   if (currentEstimatedAngle < 250)
  //   {
  //     digitalWrite(LED_PIN, HIGH);
  //   }
  //   else
  //   {
  //     digitalWrite(LED_PIN, LOW);
  //   }
  //   // Serial.print("CurrentEstimatedAngle = ");
  //   // Serial.println(currentEstimatedAngle);
  // }

  // POC with a beacon in a predefined range:

  int currentLunaDistance = getLunaDistance();
  if (currentLunaDistance < 25 && currentLunaDistance > 2)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}

std::vector<double> linspace(double start, double end, int numPoints)
{
  std::vector<double> result;

  double step = (end - start) / (numPoints - 1);
  double value = start;

  for (int i = 0; i < numPoints; ++i)
  {
    result.push_back(value);
    value += step;
  }

  return result;
}

void initSamplesBuffer()
{
  for (size_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
  {
    samplesBuffer.push(0);
  }
}

void pushSampleToQueue(int distance)
{
  samplesBuffer.pop();
  samplesBuffer.push(distance);
}

/*
  provide yaw between 0 and 359.9999 including
*/
void addMeasurmentToMapOfEnvironment(double yaw, int distance)
{
  int correspondingKey = (int)(yaw);
  mapOfEnvironment.insert(pair<int, int>(correspondingKey, distance));
}

std::vector<int> sampleMapClosestKey(const std::map<int, int> &inputMap, const std::vector<int> &keys)
{
  std::vector<int> results;

  for (int key : keys)
  {
    auto it = inputMap.lower_bound(key);

    if (it == inputMap.begin())
    {
      results.push_back(it->second);
    }
    else if (it == inputMap.end())
    {
      --it;
      results.push_back(it->second);
    }
    else
    {
      int diff1 = key - it->first;
      --it;
      int diff2 = it->first - key;

      if (diff1 <= diff2)
        ++it;

      results.push_back(it->second);
    }
  }

  return results;
}

std::vector<int> multiplyVectorByScalarAndCastToInt(const std::vector<double> &vec, double scalar)
{
  std::vector<int> result;
  result.reserve(vec.size());

  for (double element : vec)
  {
    result.push_back((int)(element * scalar));
  }

  return result;
}

std::vector<int> normalizeVectorBy360(const std::vector<int> &vec)
{
  std::vector<int> result;
  result.reserve(vec.size());

  for (int element : vec)
  {
    int normalizedValue = element % 360;
    result.push_back(normalizedValue);
  }

  return result;
}

int scalarDifference(vector<int> vec, queue<int> que)
{
  int result = 0;

  for (int element : vec)
  {
    result += abs(element - que.front());
    que.pop();
  }

  return result;
}

std::vector<int> addScalar(const std::vector<int> &vec, int scalar)
{
  std::vector<int> result;
  result.reserve(vec.size());

  for (int element : vec)
  {
    result.push_back(element + scalar);
  }

  return result;
}

bool flagMe = true;
double estimateCurrentAngle()
{
  int currentMinDifference = INT_MAX;
  double currentEstimatedAngle = -1;
  for (double rotationFreq = MIN_ESTIMATED_ROTATION_FREQ; rotationFreq <= MAX_ESTIMATED_ROTATION_FREQ; rotationFreq += ROTATION_FREQ_GRANULARITY)
  {
    double sizeOfEachSampleStep = (rotationFreq / TF_LUNA_FREQ) * 360; // in degrees
    vector<double> angles = linspace(0, (SAMPLE_BUFFER_SIZE - 1) * sizeOfEachSampleStep, SAMPLE_BUFFER_SIZE);
    // for (int i = 0; i < 360; i++)
    for (int i = 0; i < 1; i += 10)
    {
      vector<int> anglesIndexed = normalizeVectorBy360(addScalar(multiplyVectorByScalarAndCastToInt(angles, 1), i));
      vector<int> currentDistances = sampleMapClosestKey(mapOfEnvironment, anglesIndexed);

      int difference = scalarDifference(currentDistances, samplesBuffer);
      if (difference < currentMinDifference)
      {
        currentMinDifference = difference;
        currentEstimatedAngle = anglesIndexed.back();
      }
    }
  }
  Serial.print("currentMinDifference = ");
  Serial.println(currentMinDifference);
  return currentMinDifference;
  // return currentEstimatedAngle;
}

double getGyroYaw()
{
  float angleZ = mpu.getAngleZ();
  angleZ = fmod(angleZ, 360.0);
  if (angleZ < 0)
  {
    angleZ += 360.0;
  }
  return angleZ;
}

unsigned char check;
unsigned char uart[9];      /*----save data measured by LiDAR-------------*/
int rec_debug_state = 0x01; // receive state for frame
int lunaDistance;
void updateLunaDistance()
{
  if (Serial2.available()) // check if serial port has data input
  {
    if (rec_debug_state == 0x01)
    { // the first byte
      uart[0] = Serial2.read();
      if (uart[0] == 0x59)
      {
        check = uart[0];
        rec_debug_state = 0x02;
      }
    }
    else if (rec_debug_state == 0x02)
    { // the second byte
      uart[1] = Serial2.read();
      if (uart[1] == 0x59)
      {
        check += uart[1];
        rec_debug_state = 0x03;
      }
      else
      {
        rec_debug_state = 0x01;
      }
    }

    else if (rec_debug_state == 0x03)
    {
      uart[2] = Serial2.read();
      check += uart[2];
      rec_debug_state = 0x04;
    }
    else if (rec_debug_state == 0x04)
    {
      uart[3] = Serial2.read();
      check += uart[3];
      rec_debug_state = 0x05;
    }
    else if (rec_debug_state == 0x05)
    {
      uart[4] = Serial2.read();
      check += uart[4];
      rec_debug_state = 0x06;
    }
    else if (rec_debug_state == 0x06)
    {
      uart[5] = Serial2.read();
      check += uart[5];
      rec_debug_state = 0x07;
    }
    else if (rec_debug_state == 0x07)
    {
      uart[6] = Serial2.read();
      check += uart[6];
      rec_debug_state = 0x08;
    }
    else if (rec_debug_state == 0x08)
    {
      uart[7] = Serial2.read();
      check += uart[7];
      rec_debug_state = 0x09;
    }
    else if (rec_debug_state == 0x09)
    {
      uart[8] = Serial2.read();
      if (uart[8] == check)
      {
        lunaDistance = (uart[2] + uart[3] * 256); // the distance
        // strength = uart[4] + uart[5] * 256;   // the strength
        // temprature = uart[6] + uart[7] * 256; // calculate chip temprature
        // temprature = temprature / 8 - 256;
        while (Serial2.available())
        {
          Serial2.read();
        } // This part is added becuase some previous packets are there in the buffer so to clear serial buffer and get fresh data.
          // delay(100);
      }
      rec_debug_state = 0x01;
    }
  }
}

int getLunaDistance()
{
  return lunaDistance;
}

void printVector(const std::vector<int> &vec)
{
  for (int element : vec)
  {
    Serial.print(element);
    Serial.print(" ");
  }
  Serial.println("");
}

template <typename Key, typename Value>
void printMapValues(const std::map<Key, Value> &map)
{
  for (const auto &pair : map)
  {
    Serial.print(pair.second);
    Serial.print(" ");
  }
  Serial.println("");
}