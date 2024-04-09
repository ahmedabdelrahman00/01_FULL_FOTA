#include "FileSystemHandler.h"

void readFile (const char *path) {
  Serial.printf("Reading file: %s\n", path);

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return;
  }
  Serial.println("LittleFS mounted successfully");

  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
  } 
  else {
    Serial.print("File size: ");
    Serial.println(file.size());

    Serial.print("Read from file: ");
    while (file.available()) {
            byte b = file.read();
            Serial.print(String(b, HEX));
    }
  Serial.println();  
  file.close();
  }
}