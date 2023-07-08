#include "Wire.h"
#include <MPU6050_light.h>
#include <ESPAsyncWebServer.h>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <vector>
#include <sstream>

#include "FS.h"
#include "SPIFFS.h"

using namespace std;

#define FORMAT_SPIFFS_IF_FAILED false

#define LED_PIN 18
#define BUILT_IN_LED 2
#define RXD2 16
#define TXD2 17

#define TF_LUNA_FREQ 250 // in Hz

// Algorithm parameters:
#define SAMPLE_BUFFER_SIZE 30
#define SETTLE_TIME 500 // ms
#define GYROZ_THRESHOLD_FOR_ROTATION_START 600
#define ANGLE_CHECK_RANGE 3
#define ANGLE_CHECK_STRIDE 1
#define DISTANCE_SENSOR_RELATIVE_WEIGHT 1

std::vector<int> howManyTimesEachAngleWasMeasured(360, 0);
std::vector<int> mapping(360, 0);
int howManyAnglesAreDone = 0;

queue<int> samplesBuffer;

MPU6050 mpu = MPU6050(Wire);

double getGyroYaw();
int getLunaDistance();
void updateLunaDistance();
std::vector<int> linspace(double start, double end, int numPoints);
void initSamplesBuffer();
void pushSampleToQueue(int distance);
int scalarDifference(vector<int> vec, queue<int> que);
int estimateCurrentAngle();

void addVectorToLogFile(string dataName, const std::vector<int> &vec);
void readFile(fs::FS &fs, const char *path);
void appendFile(fs::FS &fs, const char *path, const char *message);
void deleteFile(fs::FS &fs, const char *path);
std::string vectorToString(const std::vector<int> &vec);
void printVector(const std::vector<int> &vec);

void setup()
{
    Serial.begin(115200); // Communication with host

    // Initialize SPIFFS
    // ==========================================================================================
    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    readFile(SPIFFS, "/log.txt");
    // // ==========================================================================================

    Serial2.begin(115200); // Communication with TF-Luna
    Wire.begin();          // For I2C
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUILT_IN_LED, OUTPUT);

    // wait until serial port opens for native USB devices
    while (!Serial)
    {
        delay(1);
    }

    // Boot MPU6050
    // ==========================================================================================
    byte status = mpu.begin(3, 3);
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
    // mpu.setFilterGyroCoef(0);
    Serial.println(F("Done!\n"));
    // ==========================================================================================

    char freq_packet[] = {0x5A, 0x06, 0x03, 0xFA, 0x00, 0x00}; // set TF-Luna freq to 250 Hz (0xFA=0d250)
    Serial2.write(freq_packet, (size_t)6);

    digitalWrite(BUILT_IN_LED, LOW);
}

double currentAngleZOffset = 0;

bool mappingCompletedFlag = true;
unsigned long prevMillisOfDistanceSample = 0;
unsigned long prevMillisOfMapping = 0;

vector<int> timestampVectorForLog;
vector<int> distanceVectorForLog;
vector<int> gyroZVectorForLog;
vector<int> estimatedAngleVectorForLog;

int rotationStartTimestamp = 0;
bool wasRotationSaved = false;

int currentEstimatedAngle = 0;

bool hasBegunRotation = false;

void loop()
{
    mpu.update();
    updateLunaDistance();

    // THE algorithm:
    // ========================================================================================
    // if (howManyAnglesAreDone < 360)
    // {
    //     if (millis() - prevMillisOfMapping >= (1000 / TF_LUNA_FREQ))
    //     {
    //         // Serial.print("TF-Luna distance(cm) = ");
    //         // Serial.println(getLunaDistance());

    //         // Serial.print("Gyro yaw (degrees) = ");
    //         // Serial.println(getGyroYaw());

    //         Serial.print("Current size of mapping: ");
    //         Serial.println(howManyAnglesAreDone);

    //         int currentAngle = int(getGyroYaw());

    //         if (howManyTimesEachAngleWasMeasured.at(currentAngle) < 5)
    //         {
    //             mapping.at(currentAngle) += getLunaDistance();
    //             if (++howManyTimesEachAngleWasMeasured.at(currentAngle) == 5)
    //             {
    //                 howManyAnglesAreDone++;
    //             }
    //         }

    //         prevMillisOfMapping = millis();
    //     }
    // }
    // else if (mappingCompletedFlag)
    // {
    //     vector<int>::iterator iter = mapping.begin();

    //     for (iter; iter < mapping.end(); iter++)
    //     {
    //         (*iter) /= 5;
    //     }

    //     mappingCompletedFlag = false;
    //     initSamplesBuffer();
    // }
    // else if (!hasBegunRotation)
    // {
    //     hasBegunRotation = mpu.getGyroZ() >= GYROZ_THRESHOLD_FOR_ROTATION_START;

    //     if (hasBegunRotation)
    //     {
    //         deleteFile(SPIFFS, "/log.txt");
    //         rotationStartTimestamp = millis();
    //         Serial.println("Logging started");
    //     }
    // }
    // else if ((millis() - prevMillisOfDistanceSample) >= (1000 / TF_LUNA_FREQ))
    // {
    //     pushSampleToQueue(getLunaDistance());
    //     if ((millis() - rotationStartTimestamp) >= SETTLE_TIME)
    //     {
    //         // int tempTime = millis();
    //         currentEstimatedAngle = estimateCurrentAngle();
    //         // Serial.print("Angle estimation took: ");
    //         // Serial.print(millis() - tempTime);
    //         // Serial.println("[ms]");
    //         if (currentEstimatedAngle >= 0 && currentEstimatedAngle <= 20)
    //         {
    //             digitalWrite(LED_PIN, LOW);
    //         }
    //         else
    //         {
    //             digitalWrite(LED_PIN, HIGH);
    //         }
    //         // Serial.print("CurrentEstimatedAngle = ");
    //         // Serial.println(currentEstimatedAngle);
    //     }

    //     if (millis() - rotationStartTimestamp < 10000) // Save data for 10 seconds
    //     {
    //         timestampVectorForLog.push_back(millis() - rotationStartTimestamp);
    //         distanceVectorForLog.push_back(getLunaDistance());
    //         gyroZVectorForLog.push_back(mpu.getGyroZ());
    //         estimatedAngleVectorForLog.push_back(currentEstimatedAngle);
    //     }
    //     else if (!wasRotationSaved)
    //     {
    //         digitalWrite(BUILT_IN_LED, HIGH);

    //         wasRotationSaved = true;
    //         addVectorToLogFile("Mapping", mapping);
    //         addVectorToLogFile("Timestamps", timestampVectorForLog);
    //         addVectorToLogFile("Distances", distanceVectorForLog);
    //         addVectorToLogFile("GyroZ", gyroZVectorForLog);
    //         addVectorToLogFile("Estimated Angle", estimatedAngleVectorForLog);

    //         Serial.println("Logging ended");
    //     }

    //     prevMillisOfDistanceSample = millis();
    // }
    // ========================================================================================

    // POC with a beacon:
    // ========================================================================================
    int currentLunaDistance = getLunaDistance();
    if (currentLunaDistance < 20)
    {
        digitalWrite(LED_PIN, LOW);
        if ((millis() - prevMillisOfDistanceSample) >= 20)
        {
            currentAngleZOffset += getGyroYaw();
            prevMillisOfDistanceSample = millis();
        }
    }
    else
    {
        digitalWrite(LED_PIN, HIGH);
    }

    if (getGyroYaw() > 50 && getGyroYaw() < 310 && !hasBegunRotation) // Detect rotation start
    {
        hasBegunRotation = true;
        deleteFile(SPIFFS, "/log.txt");
        rotationStartTimestamp = millis();
        Serial.println("Logging started");
    }

    if (hasBegunRotation && (millis() - rotationStartTimestamp < 10000) && (millis() - prevMillisOfDistanceSample) >= (1000 / TF_LUNA_FREQ)) // Save data for 10 seconds
    {
        timestampVectorForLog.push_back(millis() - rotationStartTimestamp);
        gyroZVectorForLog.push_back(getGyroYaw());
        distanceVectorForLog.push_back(getLunaDistance());
        prevMillisOfDistanceSample = millis();
    }
    else if ((millis() - rotationStartTimestamp >= 10000) && hasBegunRotation && !wasRotationSaved)
    {
        wasRotationSaved = true;
        digitalWrite(BUILT_IN_LED, HIGH);
        addVectorToLogFile("Timestamps", timestampVectorForLog);
        addVectorToLogFile("Distances", distanceVectorForLog);
        addVectorToLogFile("Current Yaw", gyroZVectorForLog);
        Serial.println("Logging ended");
    }
    // ========================================================================================

    // Led with only gyro:
    // ========================================================================================
    // int currentAngle = getGyroYaw();
    // if (currentAngle < 20 && currentAngle > 0)
    // {
    //     digitalWrite(LED_PIN, LOW);
    // }
    // else
    // {
    //     digitalWrite(LED_PIN, HIGH);
    // }
    // ========================================================================================
}

std::vector<int> linspace(double start, double end, int numPoints)
{
    std::vector<int> result;

    double step = (end - start) / (numPoints - 1);
    double value = start;

    for (int i = 0; i < numPoints; ++i)
    {
        result.push_back(int(value));
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

int scalarDifference(vector<int> vec, queue<int> que)
{
    int result = 0;

    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
        result += abs(*it - que.front());
        que.pop();
    }

    return result;
}

bool isFirstTimeEstimating = true;
unsigned long previousEstimationTimestamp = 0;
int previousEstimatedAngle = 0;
int estimateCurrentAngle()
{
    int currentEstimatedAngleBasedOnSpeedPropagation = 0;
    if (!isFirstTimeEstimating)
    {
        currentEstimatedAngleBasedOnSpeedPropagation = (int(previousEstimatedAngle + ((millis() - previousEstimationTimestamp) / 1000.0) * mpu.getGyroZ())) % 360;
    }
    previousEstimationTimestamp = millis();

    int currentMinDifference = INT_MAX;
    int currentEstimatedAngle = -1;
    double sizeOfEachSampleStep = (mpu.getGyroZ() / TF_LUNA_FREQ); // in degrees
    vector<int> angles = linspace(0, (SAMPLE_BUFFER_SIZE - 1) * sizeOfEachSampleStep, SAMPLE_BUFFER_SIZE);

    int initialI, maxI, strideI;
    if (isFirstTimeEstimating)
    {
        initialI = 0;
        maxI = 360;
        strideI = 2;

        isFirstTimeEstimating = false;
    }
    else
    {
        initialI = currentEstimatedAngleBasedOnSpeedPropagation - ANGLE_CHECK_RANGE - angles.back();
        maxI = currentEstimatedAngleBasedOnSpeedPropagation + ANGLE_CHECK_RANGE - angles.back();
        strideI = ANGLE_CHECK_STRIDE;
    }
    for (int i = initialI; i < maxI; i += strideI)
    {
        vector<int> GT_distances;

        int tempCurrentEstimatedAngle;
        for (int angle : angles)
        {
            tempCurrentEstimatedAngle = (angle + i + 360) % 360; // +360 to make sure result is positive 0-359 including
            int GT_distance = mapping.at(tempCurrentEstimatedAngle);
            GT_distances.push_back(GT_distance);
        }

        int difference = scalarDifference(GT_distances, samplesBuffer);
        if (difference < currentMinDifference)
        {
            currentMinDifference = difference;
            currentEstimatedAngle = tempCurrentEstimatedAngle;
        }
    }

    if (abs(currentEstimatedAngle - currentEstimatedAngleBasedOnSpeedPropagation) >= 180)
    {
        currentEstimatedAngle = (DISTANCE_SENSOR_RELATIVE_WEIGHT * (currentEstimatedAngle + 360) + (1 - DISTANCE_SENSOR_RELATIVE_WEIGHT) * currentEstimatedAngleBasedOnSpeedPropagation) % 360;
    }
    else
    {
        currentEstimatedAngle = DISTANCE_SENSOR_RELATIVE_WEIGHT * currentEstimatedAngle + (1 - DISTANCE_SENSOR_RELATIVE_WEIGHT) * currentEstimatedAngleBasedOnSpeedPropagation;
    }
    previousEstimatedAngle = currentEstimatedAngle;
    return currentEstimatedAngle;
}

double getGyroYaw()
{
    float angleZ = mpu.getAngleZ();
    angleZ = fmod(angleZ - currentAngleZOffset, 360.0);
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
                int tempDistance(uart[2] + uart[3] * 256);
                if (tempDistance < 800 && tempDistance > 0)
                {                                // max theoretical distance
                    lunaDistance = tempDistance; // the distance
                }
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

void addVectorToLogFile(string dataName, const std::vector<int> &vec)
{
    string dataToWrite = "\n" + dataName + ":\n==========\n" + vectorToString(vec) + "\n==========\n";
    appendFile(SPIFFS, "/log.txt", dataToWrite.c_str());
}

void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("− failed to open file for reading");
        return;
    }

    Serial.println("− read from file:");
    while (file.available())
    {
        Serial.write(file.read());
    }

    Serial.println("");

    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("− failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("− message appended");
    }
    else
    {
        Serial.println("− append failed");
    }

    file.close();
}

void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\r\n", path);
    if (fs.remove(path))
    {
        Serial.println("− file deleted");
    }
    else
    {
        Serial.println("− delete failed");
    }
}

std::string vectorToString(const std::vector<int> &vec)
{
    std::ostringstream oss;
    for (const auto &elem : vec)
    {
        oss << elem << ' ';
    }
    return oss.str();
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
