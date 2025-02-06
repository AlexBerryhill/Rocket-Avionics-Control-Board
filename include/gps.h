#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <cstddef>

class GPS {
    public:
        GPS(int tx, int rx);
        void getGPS();
    private:
        void readGPS();
};

#endif