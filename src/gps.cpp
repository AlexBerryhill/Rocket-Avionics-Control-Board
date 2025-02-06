#include "gps.h"
HardwareSerial gpsSerial(2);

GPS::GPS(int tx, int rx) {
    // code to initialize GPS
    gpsSerial.begin(9600, SERIAL_8N1, rx, tx);
    
}

void GPS::getGPS() {
    readGPS();
}

void GPS::readGPS() {
    while (gpsSerial.available() > 0){
        char c = gpsSerial.read();
        Serial.print(c);
    }
}