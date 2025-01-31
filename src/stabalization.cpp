#include "stabalization.h";

/******************************************
 * callCore
 * @brief Calls the TaskPrintCore1 function on the specified core
 * @param core: The core number to run the task on
 ******************************************/
void start(int core){ // remove parameter on arduino because of only one addressable core (there are two but it is hard to call)
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


// put function definitions here:
void moveLeft(int inches) {
  // code to move left
}