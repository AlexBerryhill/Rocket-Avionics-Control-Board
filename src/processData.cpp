#include "processData.h"
#include "gps.h"
#include <Arduino.h>

#define SD_CS 13

/******************************************
 * start
 * @brief Calls the processData function on the specified core
 * @param core: The core number to run the task on
 ******************************************/
void ProcessData::start(int core){
  xTaskCreatePinnedToCore( //xTaskCreate on Arduino
      processData,    // Task function
      "processData",      // Task name
      1024,              // Stack size
      NULL,              // Task input parameters
      1,                 // Priority (higher number = higher priority)
      NULL,              // Task handle (optional)
      core                  // Pin to Core 0 (omit on Arduino)
  );
}

void ProcessData::processData(void *pvParameters) {
  ProcessData *self = reinterpret_cast<ProcessData *>(pvParameters);
  GPS gps(1, 2);

  if (!SD.begin(SD_CS, SPI, 4000000)) {
      Serial.println("SD Card initialization failed!");
      vTaskDelete(NULL);
  }
  Serial.println("SD Card initialized.");

  for (;;) {
      for (int i = 0; i < 8; i++) {
          vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      Serial.println("GPS Data: ");
      gps.getGPS();

      self->logFile = SD.open("log.txt", FILE_APPEND);
      if (self->logFile) {
          self->logFile.println("GPS Data logged");
          self->logFile.close();
      } else {
          Serial.println("Failed to log GPS data.");
      }

      vTaskDelay((2000 - 80) / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}
