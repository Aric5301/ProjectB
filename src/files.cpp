// #include "Wire.h"
// #include <MPU6050_light.h>
// #include <cmath>
// #include <ESPAsyncWebServer.h>
// #include <iostream>
// #include <iterator>
// #include <map>
// #include <queue>

// #include <vector>
// #include <sstream>

// using namespace std;

// #include "FS.h"
// #include "SPIFFS.h"

// #define FORMAT_SPIFFS_IF_FAILED false

// void addVectorToLogFile(string dataName, const std::vector<int> &vec);
// void listDir(fs::FS &fs, const char *dirname, uint8_t levels);
// void readFile(fs::FS &fs, const char *path);
// void writeFile(fs::FS &fs, const char *path, const char *message);
// void appendFile(fs::FS &fs, const char *path, const char *message);
// void renameFile(fs::FS &fs, const char *path1, const char *path2);
// void deleteFile(fs::FS &fs, const char *path);
// std::string vectorToString(const std::vector<int> &vec);
// double getGyroYaw();
// int getLunaDistance();

// void setup()
// {
//     Serial.begin(115200); // Communication with host
//     // Initialize SPIFFS
//     // ==========================================================================================
//     if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED))
//     {
//         Serial.println("SPIFFS Mount Failed");
//         return;
//     }
//     // // ==========================================================================================

//     vector<int> vecTemp1 = {1, 3, 5, 7};
//     vector<int> vecTemp2 = {2, 4, 6, 8};

//     readFile(SPIFFS, "/log.txt");

//     addVectorToLogFile("even", vecTemp1);
//     addVectorToLogFile("odd", vecTemp2);
// }

// vector<int> timestampVectorForLog;
// vector<int> distanceVectorForLog;
// vector<int> gyroZVectorForLog;
// bool startedRotation = false;
// void loop()
// {
//     if (getGyroYaw() > 50 && getGyroYaw() < 310 && !startedRotation)
//     {
//         startedRotation = true;
//         deleteFile(SPIFFS, "/log.txt");
//     }

//     if (startedRotation)
//     {
//         timestampVectorForLog.push_back(millis());
//         distanceVectorForLog.push_back(getLunaDistance());
//         gyroZVectorForLog.push_back(getGyroYaw());
//     }
// }

// void addVectorToLogFile(string dataName, const std::vector<int> &vec)
// {
//     string dataToWrite = "\n" + dataName + ":\n==========\n" + vectorToString(vec) + "\n==========\n";
//     appendFile(SPIFFS, "/log.txt", dataToWrite.c_str());
// }

// void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
// {
//     Serial.printf("Listing directory: %s\r\n", dirname);

//     File root = fs.open(dirname);
//     if (!root)
//     {
//         Serial.println("− failed to open directory");
//         return;
//     }
//     if (!root.isDirectory())
//     {
//         Serial.println(" − not a directory");
//         return;
//     }

//     File file = root.openNextFile();
//     while (file)
//     {
//         if (file.isDirectory())
//         {
//             Serial.print("  DIR : ");
//             Serial.println(file.name());
//             if (levels)
//             {
//                 listDir(fs, file.name(), levels - 1);
//             }
//         }
//         else
//         {
//             Serial.print("  FILE: ");
//             Serial.print(file.name());
//             Serial.print("\tSIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// }

// void readFile(fs::FS &fs, const char *path)
// {
//     Serial.printf("Reading file: %s\r\n", path);

//     File file = fs.open(path);
//     if (!file || file.isDirectory())
//     {
//         Serial.println("− failed to open file for reading");
//         return;
//     }

//     Serial.println("− read from file:");
//     while (file.available())
//     {
//         Serial.write(file.read());
//     }

//     Serial.println("");

//     file.close();
// }

// void writeFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Writing file: %s\r\n", path);

//     File file = fs.open(path, FILE_WRITE);
//     if (!file)
//     {
//         Serial.println("− failed to open file for writing");
//         return;
//     }
//     if (file.print(message))
//     {
//         Serial.println("− file written");
//     }
//     else
//     {
//         Serial.println("− frite failed");
//     }

//     file.close();
// }

// void appendFile(fs::FS &fs, const char *path, const char *message)
// {
//     Serial.printf("Appending to file: %s\r\n", path);

//     File file = fs.open(path, FILE_APPEND);
//     if (!file)
//     {
//         Serial.println("− failed to open file for appending");
//         return;
//     }
//     if (file.print(message))
//     {
//         Serial.println("− message appended");
//     }
//     else
//     {
//         Serial.println("− append failed");
//     }

//     file.close();
// }

// void renameFile(fs::FS &fs, const char *path1, const char *path2)
// {
//     Serial.printf("Renaming file %s to %s\r\n", path1, path2);
//     if (fs.rename(path1, path2))
//     {
//         Serial.println("− file renamed");
//     }
//     else
//     {
//         Serial.println("− rename failed");
//     }
// }

// void deleteFile(fs::FS &fs, const char *path)
// {
//     Serial.printf("Deleting file: %s\r\n", path);
//     if (fs.remove(path))
//     {
//         Serial.println("− file deleted");
//     }
//     else
//     {
//         Serial.println("− delete failed");
//     }
// }

// std::string vectorToString(const std::vector<int> &vec)
// {
//     std::ostringstream oss;
//     for (const auto &elem : vec)
//     {
//         oss << elem << ' ';
//     }
//     return oss.str();
// }

// double getGyroYaw()
// {
//     // STUB
//     return 0;
// }