#include "stabalization.h"
#include <Arduino.h>
#include <Servo.h>
// /******************************************
//  * callCore
//  * @brief Calls the TaskPrintCore1 function on the specified core
//  * @param core: The core number to run the task on
//  ******************************************/
void Stabilization::start(int core){ // remove parameter on arduino because of only one addressable core (there are two but it is hard to call)
  myServoPitch.attach(38); // Attach pitch servo to pin 38
  myServoYaw.attach(37); // Attach pitch servo to pin 37
  myServoUp.attach(36); // Attach pitch servo to pin 36
  myServodown.attach(35); // Attach pitch servo to pin 35


  xTaskPinnedToCore( //xTaskCreate on Arduino
      stabalization,    // Task function
      "PrintCore1",      // Task name
      1024,              // Stack size
      NULL,              // Task input parameters
      1,                 // Priority (higher number = higher priority)
      NULL,              // Task handle (optional)
      core                  // Pin to Core 0 (omit on Arduino)
  ); 
}
//////////////////////////////////////////////////////////////////////////////////////////

//sample data
void ProcessData::readAndPrintMPUData(bool saveToSD) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate angles
  float currentRoll = 0.0;
  float currentPitch = 0.0;
  calculateAngles(&a, &currentRoll, &currentPitch);

  // CSV format: Timestamp, Roll, Pitch, Temperature
  char mpuDataBuffer[250];
  snprintf(mpuDataBuffer, sizeof(mpuDataBuffer), 
           "%lu,%.2f,%.2f,%.2f\n", 
           millis(), currentRoll, currentPitch, temp.temperature);
  
  Serial.print("MPU-6050 CSV: ");
  Serial.print(mpuDataBuffer);

  if (saveToSD) {
      appendFile(SD, "/mpu_log.csv", mpuDataBuffer);
  }
}



Servo myServoPitch;  // Servo for pitch control
Servo myServoYaw;    // Servo for yaw control


void setup() {
  Serial.begin(115200); // Initialize Serial Monitor for debugging
  myServoPitch.attach(9); // Attach pitch servo to pin 9
  myServoYaw.attach(10);  // Attach yaw servo to pin 10
}
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