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

// Add new methods to read sensor data
void ProcessData::readAndPrintBMPData(bool saveToSD) {
    float temperature = bmp.readTemperature();
    int32_t pressure = bmp.readPressure();
    float altitude = bmp.readAltitude();

    char bmpDataBuffer[200];
    snprintf(bmpDataBuffer, sizeof(bmpDataBuffer), 
             "BMP180 Data: Temp: %.2f°C, Pressure: %ld Pa, Altitude: %.2f m\n", 
             temperature, pressure, altitude);
    
    Serial.print(bmpDataBuffer);

    if (saveToSD) {
        appendFile(SD, "/log.txt", bmpDataBuffer);
    }
}

void ProcessData::readAndPrintMPUData(bool saveToSD) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    char mpuDataBuffer[250];
    snprintf(mpuDataBuffer, sizeof(mpuDataBuffer), 
             "MPU-6050 Data: Accel(g): X:%.2f, Y:%.2f, Z:%.2f, Gyro(°/s): X:%.2f, Y:%.2f, Z:%.2f, Temp: %.2f°C\n", 
             a.acceleration.x, a.acceleration.y, a.acceleration.z,
             g.gyro.x, g.gyro.y, g.gyro.z,
             temp.temperature);
    
    Serial.print(mpuDataBuffer);

    if (saveToSD) {
        appendFile(SD, "/log.txt", mpuDataBuffer);
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

    Serial.println("Initializing BMP180 sensor...");
    Wire.begin(SDA_PIN, SCL_PIN);
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
    createFile(SD, "/log.txt", "GPS Data: ");

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
        appendFile(SD, "/log.txt", "GPS Data: ");

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

