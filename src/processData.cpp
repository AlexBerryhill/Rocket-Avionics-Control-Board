#include "processData.h"
#include "gps.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <FS.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>

Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

#define SD_CS 13
#define SD_MOSI 12
#define SD_CLK 11
#define SD_MISO 10

#define SDA_PIN 8 
#define SCL_PIN 9

// Global variables to track angle
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Function to calculate angle from accelerometer data
void calculateAngles(sensors_event_t* a, float* roll, float* pitch) {
    // Calculate pitch (rotation around X-axis)
    *pitch = atan2(a->acceleration.y, sqrt(a->acceleration.x * a->acceleration.x + a->acceleration.z * a->acceleration.z)) * 180.0 / PI;
    
    // Calculate roll (rotation around Y-axis)
    *roll = atan2(-a->acceleration.x, a->acceleration.z) * 180.0 / PI;
}

void ProcessData::readAndPrintBMPData(bool saveToSD) {
    float temperature = bmp.readTemperature();
    int32_t pressure = bmp.readPressure();
    float altitude = bmp.readAltitude();

    // CSV format: Timestamp, Temperature, Pressure, Altitude
    char bmpDataBuffer[200];
    snprintf(bmpDataBuffer, sizeof(bmpDataBuffer), 
             "%lu,%.2f,%ld,%.2f\n", 
             millis(), temperature, pressure, altitude);
    
    Serial.print("BMP180 CSV: ");
    Serial.print(bmpDataBuffer);

    if (saveToSD) {
        appendFile(SD, "/bmp_log.csv", bmpDataBuffer);
    }
}

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

ProcessData::ProcessData() {}

ProcessData::~ProcessData() {}

void ProcessData::start(int core)
{
    xTaskCreatePinnedToCore(
        processDataStatic,  // Use a static function as the task function
        "processData",      // Task name
        8192,               // Stack size
        this,               // Pass 'this' pointer as parameter
        1,                  // Priority
        NULL,               // Task handle
        core                // Core
    );
}

// Static function that can be used with xTaskCreatePinnedToCore
void ProcessData::processDataStatic(void *pvParameters)
{
    // Cast the parameter back to a ProcessData pointer
    ProcessData *self = static_cast<ProcessData *>(pvParameters);
    
    // Now call the member function using the instance
    self->processDataInstance();
}

// Non-static member function that does the actual work
void ProcessData::processDataInstance()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to allow serial monitor to start

    Serial.println("Initializing I2C Sensors...");
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Initialize BMP180
    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP180 sensor, check wiring!");
        while (1) {}
    } else {
        Serial.println("BMP180 sensor found!");
    }

    // Initialize MPU-6050
    if (!mpu.begin()) {
        Serial.println("Could not find a valid MPU-6050 sensor, check wiring!");
        while (1) {}
    } else {
        Serial.println("MPU-6050 sensor found!");
        
        // Configure MPU-6050 settings
        mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // ±2g
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // ±250 degrees/sec
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set filter bandwidth
    }

    Serial.println("Initializing SD Card...");
    
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    bool sdInitialized = false;
    int retryCount = 0;
    while (!sdInitialized && retryCount < 3) {
        Serial.printf("SD init attempt %d/3\n", retryCount + 1);
        
        // More conservative SD initialization
        // Format: begin(ss, spi, frequency, mount_point, max_files, format_if_mount_failed)
        sdInitialized = SD.begin(SD_CS, SPI, 4000000, "/sd", 5, false);
        
        if (!sdInitialized) {
            Serial.println("SD init failed, retrying...");
            // Release SPI resources before retrying
            SD.end();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            retryCount++;
        }
    }

    if (!sdInitialized) {
        Serial.println("SD Card initialization failed after multiple attempts!");
        Serial.println("Continuing without SD card functionality");
        
        // Instead of returning, enter a safer loop that just prints data but doesn't try to use SD
        for (;;) {
            Serial.println("GPS Data: (SD disabled)");
            // gps.getGPS();
            readAndPrintBMPData(false);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        
        // We never reach this
        vTaskDelete(NULL);
    }

    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        readAndPrintBMPData(false);
        vTaskDelete(NULL);
    }

    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
        Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    Serial.println("SD Card initialized.");
    // Add CSV headers when files are created
    createFile(SD, "/bmp_log.csv", "Timestamp,Temperature,Pressure,Altitude\n");
    createFile(SD, "/mpu_log.csv", "Timestamp,Roll,Pitch,Temperature\n");

    for (;;)
    {
        for (int i = 0; i < 8; i++)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        Serial.println("GPS Data: ");
        // gps.getGPS();
        readAndPrintBMPData(true);    // Read BMP180 data and save to SD
        readAndPrintMPUData(true);    // Read MPU-6050 data and save to SD

        // Save to SD
        // appendFile(SD, "/log.txt", "GPS Data: ");

        vTaskDelay((2000 - 80) / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/********* File Management **********/
void ProcessData::createFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
}

void ProcessData::readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.println("File Content:");
    while (file.available())
    {
        Serial.write(file.read());
    }
    file.close();
}

void ProcessData::appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}

