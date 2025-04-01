#ifndef PROCESSDATA_H
#define PROCESSDATA_H
#include <Arduino.h>
#include <cstddef>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <esp_now.h>

class ProcessData {
    public:
        ProcessData();
        ~ProcessData();
        void start(int core);  
    private:
        static void processData(void *pvParameters);
        void createFile(fs::FS &fs, const char * path, const char * message);
        void appendFile(fs::FS &fs, const char * path, const char * message);
        void readFile(fs::FS &fs, const char * path);
        void readAndPrintBMPData(bool saveToSD);
        void readAndPrintMPUData(bool saveToSD);

        static void processDataStatic(void *pvParameters);
        void processDataInstance();
        void readGPS();
        static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
        void SendRadioData(char *a, float *c, float *d);


    /* data */   
};

// processData::processData(/* args */)
// {
// }

// processData::~processData()
// {
// }

#endif