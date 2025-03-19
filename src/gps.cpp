#include "gps.h"
#include <HardwareSerial.h>

HardwareSerial gpsSerial(2); // Use UART2 for GPS

GPS::GPS(int tx, int rx) {
    gpsSerial.begin(9600, SERIAL_8N1, rx, tx);  // Initialize GPS serial
}

void GPS::getGPS() {
    readGPS();
}

void GPS::readGPS() {
    while (gpsSerial.available() > 0) { // Read from GPS serial
        char c = gpsSerial.read();
        Serial.print(c);  // Print GPS data to the main serial monitor
    }
}
