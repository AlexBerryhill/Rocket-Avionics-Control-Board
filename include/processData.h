#ifndef PROCESSDATA_H
#define PROCESSDATA_H
#include <Arduino.h>
#include <cstddef>
#include <SD.h>

class ProcessData {
    public:
        void start(int core);  
    private:
        static void processData(void *pvParameters);
        File logFile;

    /* data */   
};

// processData::processData(/* args */)
// {
// }

// processData::~processData()
// {
// }

#endif