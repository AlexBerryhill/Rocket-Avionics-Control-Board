#include <Arduino.h>
// #include "stabalization.h"
// #include "processData.h"


// put function declarations here:
// void calibration();

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  // ProcessData process;
  // Stabalization stabalize;

  // Start the processing task
  // process.start(0);
  // stabalize.startStabalization(0);
  
}

void loop() {
  Serial.println("Looping pio 2...");
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  delay(1000);
  // Nothing Here
}

// void calibration() {
//   // code to calibrate
// }