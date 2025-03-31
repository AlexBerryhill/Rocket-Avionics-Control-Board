#include "stabalization.h"
#include <Arduino.h>
#include <Servo.h>
// /******************************************
//  * callCore
//  * @brief Calls the TaskPrintCore1 function on the specified core
//  * @param core: The core number to run the task on
//  ******************************************/
// void startStabalization(int core){ // remove parameter on arduino because of only one addressable core (there are two but it is hard to call)
//   xTaskPinnedToCore( //xTaskCreate on Arduino
//       stabalization,    // Task function
//       "PrintCore1",      // Task name
//       1024,              // Stack size
//       NULL,              // Task input parameters
//       1,                 // Priority (higher number = higher priority)
//       NULL,              // Task handle (optional)
//       core                  // Pin to Core 0 (omit on Arduino)
//   ); 
// }

// void TaskPrintCore1(void *pvParameters) {
//   for(;;) {
//     // code to stabilize
//     // stabalization();
    
//     Serial.println("Task 1 running on core " + xPortGetCoreID());
//     delay(1000);
//   }

//   vTaskDelete(NULL);
// }

// /******************************************
//  * stabalization
//  * @brief Stabalizes the robot
//  ******************************************/

Servo myServoPitch;  // Servo for pitch control
Servo myServoYaw;    // Servo for yaw control

// int pitchBefore = 90;  // Initial pitch value
// int pitchAfter = 100;  // Example pitch value
// int YawBefore = 90;    // Initial yaw value
// int yawNow = 120;      // Example yaw value

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor for debugging
  myServoPitch.attach(9); // Attach pitch servo to pin 9
  myServoYaw.attach(10);  // Attach yaw servo to pin 10
}

void loop() {
  // Update pitch servo
  updateServoPitch(pitchBefore, pitchNow);

  // Update yaw servo
  updateServoYaw(YawBefore, yawNow);

  delay(100); // Optional delay to prevent rapid updates
}

// Function to update pitch servo
void updateServoPitch(int &pitchBefore, int pitchNow) {
  if (pitchBefore != pitchNow) {
    int servoAnglePitch = constrain(pitchNow, 0, 180); // Ensure valid angle range
    myServoPitch.write(servoAnglePitch); // Update pitch servo position
    pitchBefore = pitchNow;  // Update pitch reference value
    Serial.print("Pitch Updated to: ");
    Serial.println(servoAnglePitch);
  }
}

// Function to update yaw servo
void updateServoYaw(int &YawBefore, int yawNow) {
  if (YawBefore != yawNow) {
    int servoAngleYaw = constrain(yawNow, 0, 180); // Ensure valid angle range
    myServoYaw.write(servoAngleYaw); // Update yaw servo position
    YawBefore = yawNow;  // Update yaw reference value
    Serial.print("Yaw Updated to: ");
    Serial.println(servoAngleYaw);
  }
}

// /******************************************
//  * moveLeft
//  * @brief Moves the robot left by the specified number of inches
//  * @param inches: The number of inches to move left
//  ******************************************/
// void moveLeft(int inches) {
//   // code to move left
// }