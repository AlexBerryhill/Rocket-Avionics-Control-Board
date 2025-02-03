#include "stabalization.h";

/******************************************
 * callCore
 * @brief Calls the TaskPrintCore1 function on the specified core
 * @param core: The core number to run the task on
 ******************************************/
void startStabalization(int core){ // remove parameter on arduino because of only one addressable core (there are two but it is hard to call)
  xTaskPinnedToCore( //xTaskCreate on Arduino
      TaskPrintCore1,    // Task function
      "PrintCore1",      // Task name
      1024,              // Stack size
      NULL,              // Task input parameters
      1,                 // Priority (higher number = higher priority)
      NULL,              // Task handle (optional)
      core                  // Pin to Core 0 (omit on Arduino)
  ); 
}

void TaskPrintCore1(void *pvParameters) {
  for(;;) {
    // code to stabilize
    // stabalization();
    
    Serial.println("Task 1 running on core " + xPortGetCoreID());
    delay(1000);
  }

  vTaskDelete(NULL);
}

/******************************************
 * stabalization
 * @brief Stabalizes the robot
 ******************************************/
void stabalization() {
  // code to stabilize
}

/******************************************
 * moveLeft
 * @brief Moves the robot left by the specified number of inches
 * @param inches: The number of inches to move left
 ******************************************/
void moveLeft(int inches) {
  // code to move left
}