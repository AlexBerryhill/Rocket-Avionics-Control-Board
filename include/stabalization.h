#ifndef STABALIZATION_H
#define STABALIZATION_H
#include <Arduino.h>
#include <cstddef>

// Define the prototypes for the functions
class Stabalization
{
    public:
        void start(int core);
    private:
        void stabalization();
};

#endif

