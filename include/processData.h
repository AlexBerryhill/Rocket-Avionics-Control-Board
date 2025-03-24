#ifndef PROCESSDATA_H
#define PROCESSDATA_H
#include <Arduino.h>
#include <cstddef>
#include <SPI.h>
#include <FS.h>
#include <SD.h>

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

        static void processDataStatic(void *pvParameters);
        void processDataInstance();

    /* data */   
};

// processData::processData(/* args */)
// {
// }

// processData::~processData()
// {
// }

#endif