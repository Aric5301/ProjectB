// #include "Adafruit_VL53L0X.h"
// #include "Wire.h"
// #include <MPU6050_light.h>
// #include <cmath>
// #include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>

// #include <iostream>
// #include <iterator>
// #include <map>
// #include <queue>
// using namespace std;

// #define MIN_ESTIMATED_ROTATION_FREQ 0.1 // in Hz
// #define MAX_ESTIMATED_ROTATION_FREQ 10  // in Hz
// #define ROTATION_FREQ_GRANULARITY 0.1   // in Hz
// #define TF_LUNA_FREQ 250                // in Hz
// #define SAMPLE_BUFFER_SIZE 10

// std::map<int, int> mapOfEnvironment; // maps half degrees to distances. key=0 is angle 0, key=1 is angle 0.5, key=2 is angle 1, ... key=719 is 359.5.
// queue<int> samplesBuffer;

// std::vector<double> linspace(double start, double end, int numPoints)
// {
//   std::vector<double> result;

//   double step = (end - start) / (numPoints - 1);
//   double value = start;

//   for (int i = 0; i < numPoints; ++i)
//   {
//     result.push_back(value);
//     value += step;
//   }

//   return result;
// }

// void initSamplesBuffer()
// {
//   for (size_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
//   {
//     samplesBuffer.push(0);
//   }
// }

// void pushSampleToQueue(int distance)
// {
//   samplesBuffer.pop();
//   samplesBuffer.push(distance);
// }

// /*
//   provide yaw between 0 and 359.9999 including
// */
// void addMeasurmentToMapOfEnvironment(double yaw, int distance)
// {
//   int correspondingKey = (int)(yaw * 2);
//   mapOfEnvironment.insert(pair<int, int>(correspondingKey, distance));
// }

// std::vector<int> sampleMapClosestKey(const std::map<int, int> &inputMap, const std::vector<int> &keys)
// {
//   std::vector<int> results;

//   for (int key : keys)
//   {
//     auto it = inputMap.lower_bound(key);

//     if (it == inputMap.begin())
//     {
//       results.push_back(it->second);
//     }
//     else if (it == inputMap.end())
//     {
//       --it;
//       results.push_back(it->second);
//     }
//     else
//     {
//       int diff1 = key - it->first;
//       --it;
//       int diff2 = it->first - key;

//       if (diff1 <= diff2)
//         ++it;

//       results.push_back(it->second);
//     }
//   }

//   return results;
// }

// std::vector<int> multiplyVectorByScalarAndCastToInt(const std::vector<double> &vec, double scalar)
// {
//   std::vector<int> result;
//   result.reserve(vec.size());

//   for (double element : vec)
//   {
//     result.push_back((int)(element * scalar));
//   }

//   return result;
// }

// std::vector<int> normalizeVectorBy720(const std::vector<int> &vec)
// {
//   std::vector<int> result;
//   result.reserve(vec.size());

//   for (int element : vec)
//   {
//     int normalizedValue = element % 720;
//     result.push_back(normalizedValue);
//   }

//   return result;
// }

// int scalarMultiply(vector<int> vec, queue<int> que)
// {
//   int result = 0;

//   for (int element : vec)
//   {
//     result += element * que.front();
//     que.pop();
//   }

//   return result;
// }

// std::vector<int> addScalar(const std::vector<int> &vec, int scalar)
// {
//   std::vector<int> result;
//   result.reserve(vec.size());

//   for (int element : vec)
//   {
//     result.push_back(element + scalar);
//   }

//   return result;
// }

// double estimateCurrentAngle()
// {
//   int currentMaxCorrelation = 0;
//   double currentEstimatedAngle = -1;
//   for (double rotationFreq = MIN_ESTIMATED_ROTATION_FREQ; rotationFreq <= MAX_ESTIMATED_ROTATION_FREQ; rotationFreq += ROTATION_FREQ_GRANULARITY)
//   {
//     double sizeOfEachSampleStep = (rotationFreq / TF_LUNA_FREQ) * 360; // in degrees
//     vector<double> angles = linspace(0, (SAMPLE_BUFFER_SIZE - 1) * sizeOfEachSampleStep, SAMPLE_BUFFER_SIZE);
//     for (int i = 0; i < 720; i++)
//     {
//       vector<int> anglesInHalfDegreesIndexed = normalizeVectorBy720(addScalar(multiplyVectorByScalarAndCastToInt(angles, 2), i));
//       vector<int> currentDistances = sampleMapClosestKey(mapOfEnvironment, anglesInHalfDegreesIndexed);
//       int correlation = scalarMultiply(currentDistances, samplesBuffer);
//       if (correlation > currentMaxCorrelation)
//       {
//         currentMaxCorrelation = correlation;
//         currentEstimatedAngle = anglesInHalfDegreesIndexed.back() / 2.0;
//       }
//     }
//   }
//   return currentEstimatedAngle;
// }
