#include <cstddef>
#include <cstdint>
#include <Wire.h>

//The Gyro class implements the GY-521 MPU6050 6DOF IMU sensor
class Gyro {
    public:
        //Constructor
        Gyro(){
            wire.begin();
        };
        //Destructor
        ~Gyro(){
            wire.end();
        };
        const int MPU6050_ADDR = 0x68;
        int ACC_X;;
        int ACC_Y;
        int ACC_Z;
        int GYRO_X;
        int GYRO_Y;
        int GYRO_Z;

        //Reads the raw accelerometer data
        void readAccelRaw(int16_t* accel){
            
        };
        //Reads the raw gyroscope data
        void readGyroRaw(int16_t* gyro);
        //Reads the raw temperature data
        void readTempRaw(int16_t* temp);
        //Reads the accelerometer data in m/s^2
        void readAccel(float* accel);
        //Reads the gyroscope data in rad/s
        void readGyro(float* gyro);
        //Reads the temperature data in degrees Celsius
        void readTemp(float* temp);

}