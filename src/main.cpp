#include <Arduino.h>
#include <SPI.h>
// #include "stabalization.h"
// #include "processData.h"


// put function declarations here:
// void calibration();
#define SD_CS 13
#define SD_MOSI 12
#define SD_CLK 11
#define SD_MISO 10

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);

  // ProcessData process;
  // Stabalization stabalize;

  // Start the processing task
  // process.start(0);
  // stabalize.startStabalization(0);
  
}

void loop() {
  // Serial.println("Looping pio 2...");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  // delay(1000);
  // Nothing Here
}

// void calibration() {
//   // code to calibrate
// }