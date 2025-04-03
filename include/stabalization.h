#ifndef STABALIZATION_H
#define STABALIZATION_H
#include <Arduino.h>
#include <cstddef>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>

// Define the prototypes for the functions
class Stabalization
{
    public:
        void start(int core);
    private:
        void stabalization();
        static void calculateAngles(sensors_event_t* a, float* roll, float* pitch);
        void updateAngles();
        static void updateServoPitch(float pitchNow);
        static void updateServoYaw(float yawNow);
        static void stabalizationWrapper(void* pvParameters);
};

#endif

