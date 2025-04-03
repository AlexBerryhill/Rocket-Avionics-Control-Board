#include "stabalization.h"
#include <Arduino.h>
#include <ESP32Servo.h>


#define SDA_PIN 8 
#define SCL_PIN 9

#ifndef MPU
#define MPU
Adafruit_MPU6050 mpu2;
#endif

Servo myServoPitch;  // Servo for pitch control
Servo myServoYaw;    // Servo for yaw control
Servo myServoUp;  // Servo for pitch control
Servo myServoDown;    // Servo for yaw control
float YawBefore = 90.0; // Previous yaw angle
float pitchBefore = 90.0; // Previous roll angle
// float yawNow = 90.0; // Current yaw angle
// float pitchNow = 90.0; // Current roll angle

// /******************************************
//  * callCore
//  * @brief Calls the Stabalization function on the specified core
//  * @param core: The core number to run the task on
//  ******************************************/
void Stabalization::start(int core){ // remove parameter on arduino because of only one addressable core (there are two but it is hard to call)
  myServoPitch.attach(24); // Attach pitch servo to pin 35
  myServoYaw.attach(26); // Attach pitch servo to pin 37
  myServoUp.attach(25); // Attach pitch servo to pin 36
  myServoDown.attach(38); // Attach pitch servo to pin 38


  Serial.println("Creating stabalization task...");
        xTaskCreatePinnedToCore(
            stabalizationWrapper,  // Static wrapper function
            "StabalizationTask",   // Task name
            8192,                  // Stack size (bytes)
            this,                  // Pass 'this' as parameter
            2,                     // Priority
            NULL,                  // Task handle
            core 
        );
}
//////////////////////////////////////////////////////////////////////////////////////////

// Static wrapper function for FreeRTOS
void Stabalization::stabalizationWrapper(void* pvParameters) {
  Stabalization* instance = static_cast<Stabalization*>(pvParameters);
  instance->stabalization();  // Call the member function
}

// Function to calculate angle from accelerometer data
void Stabalization::calculateAngles(sensors_event_t* a, float* roll, float* pitch) {
    // Calculate pitch (rotation around X-axis)
    *pitch = atan2(a->acceleration.y, sqrt(a->acceleration.x * a->acceleration.x + a->acceleration.z * a->acceleration.z)) * 180.0 / PI;
    
    // Calculate roll (rotation around Y-axis)
    *roll = atan2(-a->acceleration.x, a->acceleration.z) * 180.0 / PI;

    // Calculate yaw (rotation around Z-axis)
    // *yaw = atan2(a->acceleration.y, a->acceleration.x) * 180.0 / PI;
}

//sample data
void Stabalization::updateAngles() {
  sensors_event_t a, g, temp;
  mpu2.getEvent(&a, &g, &temp);

  // Calculate angles
  float currentRoll = 0.0;
  float currentPitch = 0.0;
  calculateAngles(&a, &currentRoll, &currentPitch);


}


/******************************************
 * stabalization
 * @brief Task responsible for stabilizing the rocket by reading sensor data and updating servo positions.
 *
 * This function initializes the I2C sensors, including the MPU-6050, and configures its settings.
 * It continuously reads accelerometer and gyroscope data, calculates the current roll and pitch angles,
 * and updates the pitch and yaw servos accordingly to stabilize the rocket.
 *
 * @param pvParameters Pointer to task parameters (unused in this implementation).
 *
 * @note The function runs as a FreeRTOS task and includes a delay of 50ms between iterations.
 *       It will block indefinitely if the MPU-6050 sensor is not found.
 *       Ensure that the `updateServoPitch` and `updateServoYaw` methods are implemented to avoid linker errors.
 */
void Stabalization::stabalization() {
    Serial.println("Initializing I2C Sensors...");
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialize MPU6050
    if (!mpu2.begin()) {
        Serial.println("Could not find a valid MPU-6050 sensor, check wiring!");
        while (1) {}
    } else {
        Serial.println("MPU-6050 sensor found!");
        
        // Configure MPU-6050 settings
        mpu2.setAccelerometerRange(MPU6050_RANGE_2_G);  // ±2g
        mpu2.setGyroRange(MPU6050_RANGE_250_DEG);       // ±250 degrees/sec
        mpu2.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set filter bandwidth
    }

    // Update pitch servo
    for(;;){

        // Read MPU data
        sensors_event_t a, g, temp;
        mpu2.getEvent(&a, &g, &temp);
        // Calculate angles
        float currentRoll = 0.0;
        float currentPitch = 0.0;
        calculateAngles(&a, &currentRoll, &currentPitch);


        updateServoPitch(currentPitch);

        // Update yaw servo
        updateServoYaw(currentRoll);

        vTaskDelay(50 / portTICK_PERIOD_MS); // Delay for 50ms
    }
    vTaskDelete(NULL);
}

// Function to update pitch servo
void Stabalization::updateServoPitch(float pitchNow) {
  if (pitchBefore != pitchNow) {
    Serial.print("Pitch: ");
    Serial.println(pitchNow);

    float servoAnglePitch = constrain(pitchNow, 0, 180); // Ensure valid angle range
    myServoPitch.write(servoAnglePitch); // Update pitch servo position
    pitchBefore = pitchNow;  // Update pitch reference value
    Serial.print("Pitch Updated to: ");
    Serial.println(servoAnglePitch);
  }
}

// Function to update yaw servo
void Stabalization::updateServoYaw(float yawNow) {
  if (YawBefore != yawNow) {
    float servoAngleYaw = constrain(yawNow, 0, 180); // Ensure valid angle range
    myServoYaw.write(servoAngleYaw); // Update yaw servo position
    YawBefore = yawNow;  // Update yaw reference value
    Serial.print("Yaw Updated to: ");
    Serial.println(servoAngleYaw);
  }
}

