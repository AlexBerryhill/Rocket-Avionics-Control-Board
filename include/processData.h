#ifndef PROCESSDATA_H
#define PROCESSDATA_H
#include <Arduino.h>
#include <cstddef>

class ProcessData {
    public:
        void start(int core);  
    private:
        static void processData(void *pvParameters);
    /* data */   
};

// processData::processData(/* args */)
// {
// }

// processData::~processData()
// {
// }

#endif