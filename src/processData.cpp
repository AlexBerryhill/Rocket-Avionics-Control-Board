#include "processData.h"
#include "gps.h"
#include <Arduino.h>

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
//   int pressureList[8];
  GPS gps(1, 2);

  for(;;) {
    // Empty the pressure list
    // pressureList.empty_slots();
    // code to stabilize
    for (int i = 0; i < 8; i++) { // Takes 80 ms to read pressure sensor 8 times
        // Add to pressure list
        // pressureList.push_back(pressureSensor.readPressure());
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Get GPS
    gps.getGPS();
    // Save data to SD and Radio
    // saveSD(pressureList);
    // sendRadio(pressureList);

    // vTaskDelay((2000-80) / portTICK_PERIOD_MS);
  }

  // vTaskDelete(NULL);
}