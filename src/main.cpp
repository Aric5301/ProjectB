#include "Wire.h"
#include <MPU6050_light.h>
#include <ESPAsyncWebServer.h>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <vector>
#include <sstream>
#include <algorithm>

#include "FS.h"
#include "SPIFFS.h"

using namespace std;

#define FORMAT_SPIFFS_IF_FAILED false

#define LED_PIN 18
#define BUILT_IN_LED 2
#define RXD2 16
#define TXD2 17

// Algorithm parameters:

// change this to choose whether you want the main algorithm active or only the beacon algorithm active.
#define IS_ALGORITHM_USED true

#define SHOULD_PRINT_PREV_LOG_ON_BOOT true

#define MAPPING_BUFFER_SIZE 3
#define SAMPLE_BUFFER_SIZE 30
#define SETTLE_TIME 0.5                        // seconds
#define GYROZ_THRESHOLD_FOR_ROTATION_START 600 // degrees/sec
#define ANGLE_CHECK_RANGE 3
#define ANGLE_CHECK_STRIDE 1
#define DISTANCE_SENSOR_RELATIVE_WEIGHT 1

#define BEACON_DISTANCE 20
// ==========================================================================================

std::vector<int> howManyTimesEachAngleWasMeasured(360, 0);
std::vector<int> mapping(360, 0);
int howManyAnglesAreDone = 0;

MPU6050 mpu = MPU6050(Wire);

bool performMapping();
bool updateLunaDistance();
bool updateAllData();
int estimateCurrentAngle();

void addVectorToLogFile(string dataName, const std::vector<int> &vec);
void readFile(fs::FS &fs, const char *path);
void appendFile(fs::FS &fs, const char *path, const char *message);
void deleteFile(fs::FS &fs, const char *path);
std::string vectorToString(const std::vector<int> &vec);

void setup()
{
    Serial.begin(115200);  // Communication with host
    Serial2.begin(115200); // Communication with TF-Luna
    Wire.begin();          // For I2C

    // Initialize LED ports
    // ------------------------------------------------------------------------------------------
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUILT_IN_LED, OUTPUT);
    // ==========================================================================================

    // wait until serial port opens for native USB devices
    // ------------------------------------------------------------------------------------------
    while (!Serial)
    {
        delay(1);
    }
    // ==========================================================================================

    // Initialize SPIFFS
    // ==========================================================================================
    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    if (SHOULD_PRINT_PREV_LOG_ON_BOOT)
    {
        readFile(SPIFFS, "/log.txt");
    }
    // // ==========================================================================================

    // Boot MPU6050
    // ------------------------------------------------------------------------------------------
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
    mpu.calcOffsets();
    Serial.println(F("Done!\n"));
    // ==========================================================================================

    // Initialize TF-Luna
    // ------------------------------------------------------------------------------------------
    char freq_packet[] = {0x5A, 0x06, 0x03, 0xFA, 0x00, 0x00}; // set TF-Luna freq to 250 Hz (0xFA=0d250)
    Serial2.write(freq_packet, (size_t)6);
    // ==========================================================================================

    // Default state for LEDs
    // ------------------------------------------------------------------------------------------
    digitalWrite(BUILT_IN_LED, HIGH);
    digitalWrite(LED_PIN, HIGH);
    // ==========================================================================================
}

enum CurrentState
{
    state_mapping,
    state_waiting_to_rotate,
    state_rotating_and_recording,
    state_done
};

vector<int> timestampsVector;                     // stores timestamps of algo runs. Units: micro-seconds
vector<int> distancesVector;                      // stores distances from TF-Luna samples. Units: cm
vector<int> gyroZVector;                          // stores angular velocity reads from gyro. Units: degrees/second.
vector<int> angleStepsVector;                     // stores sizes of angular step since last measurment. How many degrees has the system passed since last time. Units: degrees.
vector<int> estimatedAngleMainAlgoVectorForLog;   // stores the estimated angle as computed by the main algorithm. Units: whole degrees [0-359]
vector<int> estimatedAngleBeaconAlgoVectorForLog; // stores the estimated angle as computed by the beacon algorithm. Units: whole degrees [0-359]
vector<int> estimatedAngleNoAlgoVectorForLog;     // stores the estimated angle just as an integral on gyroZ. Units: whole degrees [0-359]

unsigned long rotationStartTimestamp;
unsigned long currentTimestamp;
unsigned long lastTimestamp;
int currentLunaDistance;
float currentGyroZ;
float currentNormalizedAngleZ;
float currentNormalizedAngleZWithOffset;
float currentAngleStep;
CurrentState currentState = state_mapping;

float currentAngleZOffset = 0;

void loop()
{
    if (!updateAllData())
    {
        return; // Don't do anything if there isn't a new distance sample available
    }

    switch (currentState)
    {
    case state_mapping:
        if (!IS_ALGORITHM_USED || performMapping())
        {
            currentState = state_waiting_to_rotate;
        }
        break;
    case state_waiting_to_rotate:
        if (currentGyroZ >= GYROZ_THRESHOLD_FOR_ROTATION_START)
        {
            deleteFile(SPIFFS, "/log.txt");
            rotationStartTimestamp = currentTimestamp;
            Serial.println("Logging started");
            digitalWrite(BUILT_IN_LED, LOW);

            currentState = state_rotating_and_recording;
        }
        break;
    case state_rotating_and_recording:
        if (currentLunaDistance <= BEACON_DISTANCE)
        {
            currentAngleZOffset = currentNormalizedAngleZ;
        }

        timestampsVector.push_back((currentTimestamp - rotationStartTimestamp) / 1000);
        distancesVector.push_back(currentLunaDistance);
        gyroZVector.push_back(currentGyroZ);
        angleStepsVector.push_back(currentAngleStep);
        estimatedAngleBeaconAlgoVectorForLog.push_back(currentNormalizedAngleZWithOffset);
        estimatedAngleNoAlgoVectorForLog.push_back(currentNormalizedAngleZ);

        int currentEstimatedAngle;
        if (IS_ALGORITHM_USED)
        {
            currentEstimatedAngle = estimateCurrentAngle();
            estimatedAngleMainAlgoVectorForLog.push_back(currentEstimatedAngle);
        }
        else
        {
            currentEstimatedAngle = currentNormalizedAngleZWithOffset;
        }

        if (currentEstimatedAngle <= 10 || currentEstimatedAngle >= 350)
        {
            digitalWrite(LED_PIN, LOW); // Since the LED is connect in a sink configuration, setting the port to LOW means turning it on
        }
        else
        {
            digitalWrite(LED_PIN, HIGH);
        }

        if (micros() - rotationStartTimestamp >= 10000000) // stop saving data after 10 seconds
        {
            digitalWrite(BUILT_IN_LED, HIGH);

            addVectorToLogFile("Mapping", mapping);
            addVectorToLogFile("Timestamps", timestampsVector);
            addVectorToLogFile("Distances", distancesVector);
            addVectorToLogFile("GyroZ", gyroZVector);
            addVectorToLogFile("Estimated Angle Main Algo", estimatedAngleMainAlgoVectorForLog);
            addVectorToLogFile("Estimated Angle Beacon Algo", estimatedAngleBeaconAlgoVectorForLog);
            addVectorToLogFile("Estimated Angle No Algo", estimatedAngleNoAlgoVectorForLog);

            Serial.println("Logging ended");

            currentState = state_done;
        }
        break;
    case state_done:
        break;
    }
}

// returns true when mapping is done
bool performMapping()
{
    if (howManyAnglesAreDone < 360)
    {
        if (howManyAnglesAreDone >= 5)
        {
            digitalWrite(BUILT_IN_LED, LOW);
        }

        Serial.print("Current size of mapping: ");
        Serial.println(howManyAnglesAreDone);

        if (howManyTimesEachAngleWasMeasured.at(int(currentNormalizedAngleZ)) < MAPPING_BUFFER_SIZE)
        {
            mapping.at(int(currentNormalizedAngleZ)) += currentLunaDistance;
            if (++howManyTimesEachAngleWasMeasured.at(int(currentNormalizedAngleZ)) == MAPPING_BUFFER_SIZE)
            {
                howManyAnglesAreDone++;
            }
        }
    }
    else
    {
        vector<int>::iterator iter = mapping.begin();

        for (iter; iter < mapping.end(); iter++)
        {
            (*iter) /= MAPPING_BUFFER_SIZE;
        }

        digitalWrite(BUILT_IN_LED, HIGH);

        return true;
    }
    return false;
}

// returns true if new distance sample is available
bool updateAllData()
{
    bool output;

    mpu.update();

    currentTimestamp = micros();

    currentGyroZ = mpu.getGyroZ();

    currentNormalizedAngleZ = mpu.getAngleZ();
    currentNormalizedAngleZ = fmod(currentNormalizedAngleZ, 360.0);
    if (currentNormalizedAngleZ < 0)
    {
        currentNormalizedAngleZ += 360.0;
    }

    currentNormalizedAngleZWithOffset = currentNormalizedAngleZ - currentAngleZOffset;
    currentNormalizedAngleZWithOffset = fmod(currentNormalizedAngleZWithOffset, 360.0);
    if (currentNormalizedAngleZWithOffset < 0)
    {
        currentNormalizedAngleZWithOffset += 360.0;
    }

    output = updateLunaDistance(); // updates currentLunaDistance variable

    currentAngleStep = ((currentTimestamp - lastTimestamp) * currentGyroZ) / 1000000.0;

    lastTimestamp = currentTimestamp;

    return output;
}

bool isFirstTimeEstimating = true;
int previousEstimatedAngle = 0;
unsigned long previousEstimationTimestamp = 0;
int estimateCurrentAngle()
{
    // Wait for rotation to stabilize
    if (currentTimestamp - rotationStartTimestamp <= SETTLE_TIME * 1000000)
    {
        return 0;
    }

    int currentEstimatedAngleBasedOnSpeedPropagation = 0;
    if (!isFirstTimeEstimating)
    {
        currentEstimatedAngleBasedOnSpeedPropagation = (int(previousEstimatedAngle + ((currentTimestamp - previousEstimationTimestamp) / 1000000.0) * currentGyroZ)) % 360;
    }
    previousEstimationTimestamp = currentTimestamp;

    int currentMinDifference = INT_MAX;
    int currentEstimatedAngle = -1;
    size_t angleStepsVectorSize = angleStepsVector.size();
    vector<int> angles;
    angles.push_back(0);

    for (size_t i = SAMPLE_BUFFER_SIZE - 1; i > 0; i--)
    {
        angles.push_back(angleStepsVector.at(angleStepsVectorSize - i));
    }

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
        vector<int> GT_distances; // GT stands for ground truth

        int tempCurrentEstimatedAngle;
        for (int angle : angles)
        {
            tempCurrentEstimatedAngle = (angle + i + 360) % 360; // +360 to make sure result is positive 0-359 including
            int GT_distance = mapping.at(tempCurrentEstimatedAngle);
            GT_distances.push_back(GT_distance);
        }

        int currentDifference = 0;
        size_t distancesVectorSize = distancesVector.size();
        for (size_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
        {
            currentDifference += abs(GT_distances.at(i) - distancesVector.at(distancesVectorSize - 1 - i));
        }

        if (currentDifference <= currentMinDifference)
        {
            currentMinDifference = currentDifference;
            currentEstimatedAngle = tempCurrentEstimatedAngle;
        }
    }

    currentEstimatedAngleBasedOnSpeedPropagation = (currentEstimatedAngleBasedOnSpeedPropagation + 360) % 360;
    if (abs(currentEstimatedAngle - currentEstimatedAngleBasedOnSpeedPropagation) >= 180)
    {
        if (currentEstimatedAngle < currentEstimatedAngleBasedOnSpeedPropagation)
        {
            currentEstimatedAngle = (int)(DISTANCE_SENSOR_RELATIVE_WEIGHT * (currentEstimatedAngle + 360) + (1 - DISTANCE_SENSOR_RELATIVE_WEIGHT) * currentEstimatedAngleBasedOnSpeedPropagation) % 360;
        }
        else
        {
            currentEstimatedAngle = (int)(DISTANCE_SENSOR_RELATIVE_WEIGHT * currentEstimatedAngle + (1 - DISTANCE_SENSOR_RELATIVE_WEIGHT) * (currentEstimatedAngleBasedOnSpeedPropagation + 360)) % 360;
        }
    }
    else
    {
        currentEstimatedAngle = DISTANCE_SENSOR_RELATIVE_WEIGHT * currentEstimatedAngle + (1 - DISTANCE_SENSOR_RELATIVE_WEIGHT) * currentEstimatedAngleBasedOnSpeedPropagation;
    }
    previousEstimatedAngle = currentEstimatedAngle;

    return currentEstimatedAngle;
}

bool updateLunaDistance() // Returns true if new reading was successfully read, otherwise returns false
{
    bool output = false;

    uint8_t buffer[256];
    unsigned char check = 0;

    size_t howMuchRead = Serial2.read(buffer, 256);
    for (size_t packet_i = 1; packet_i <= (howMuchRead / 9); packet_i++)
    {
        uint8_t *packet = buffer + howMuchRead - 9 * packet_i;

        for (size_t i = 0; i <= 7; i++)
        {
            check += packet[i];
        }

        if (packet[0] == 0x59 && packet[1] == 0x59 && packet[8] == check)
        {

            int tempDistance(packet[2] + packet[3] * 256);
            if (tempDistance < 800 && tempDistance > 0)
            {
                currentLunaDistance = tempDistance; // the distance
                return true;
            }
        }
    }

    return false;
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
