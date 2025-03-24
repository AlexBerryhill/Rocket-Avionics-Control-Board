#include <Arduino.h>
#include <SPI.h>
// #include "stabalization.h"
#include "processData.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  ProcessData process;
  // Stabalization stabalize;

  // Start the processing task
  process.start(1);
  // stabalize.startStabalization(0);
  
}

void loop() {
  // Serial.println("Looping pio 2...");
  vTaskDelay(portMAX_DELAY);
}

// void calibration() {
//   // code to calibrate
// }