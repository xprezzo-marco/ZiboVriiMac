//
//  SerialPort.cpp
//  ZiboMacVriiPlugin
//
//

#include "SerialPort.hpp"

#include <chrono>
#include <fstream>
#include <string>

#include <fcntl.h> //open()
#include <stdio.h>
#include <termios.h>
#include <unistd.h> //write(), read(), close()
#include <errno.h> //errno
#include <cstring>
#include "logger.h"


using namespace std::chrono;


#define SERIAL_CONFIG_FILE "./Resources/plugins/ZiboVrII/config/serialport"

//Serial port file descriptor
static const int SFD_UNAVAILABLE = -1;
static int sfd = SFD_UNAVAILABLE;
#define DEFAULT_PORT "/dev/tty.usbserial-A6009aIw";

const char* portPath = DEFAULT_PORT;

int openAndConfigureSerialPort(const char* portPath, int baudRate) {
    
    //If port is already open, close it
    if (serialPortIsOpen()) {
        close(sfd);
    }
    
    //Open port, checking for errors
    sfd = open(portPath, (O_RDWR | O_NOCTTY | O_NDELAY));
    if (sfd == -1) {
        printf("Unable to open serial port: %s at baud rate: %d\n", portPath, baudRate);
        return sfd;
    }
    
  
    
    //Configure i/o baud rate settings
    struct termios options;
    tcgetattr(sfd, &options);
    switch (baudRate) {
        case 9600:
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
            break;
        case 19200:
            cfsetispeed(&options, B19200);
            cfsetospeed(&options,B19200);
            break;
        case 38400:
            cfsetispeed(&options, B38400);
            cfsetospeed(&options, B38400);
            break;
        case 57600:
            cfsetispeed(&options, B57600);
            cfsetospeed(&options, B57600);
            break;
        case 115200:
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            break;
        default:
            printf("Requested baud rate %d not currently supported. Defaulting to 9,600.\n", baudRate);
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
            break;
    }
    
    //Configure other settings
    //Settings from:
    //  https://github.com/Marzac/rs232/blob/master/rs232-linux.c
    //
    options.c_iflag &= ~(INLCR | ICRNL);
    options.c_iflag |= IGNPAR | IGNBRK;
    options.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    options.c_cflag &= ~(PARENB | PARODD | CSTOPB | CSIZE | CRTSCTS);
    options.c_cflag |= CLOCAL | CREAD | CS8;
    options.c_lflag &= ~(ICANON | ISIG | ECHO);
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN]  = 0;
    
    //Apply settings
    //TCSANOW vs TCSAFLUSH? Was using TCSAFLUSH; settings source above
    //uses TCSANOW.
    if (tcsetattr(sfd, TCSANOW, &options) < 0) {
        VLError("Error setting serial port attributes.");
        close(sfd);
        return -2; //Using negative value; -1 used above for different failure
    }
    
    return sfd;
}



bool serialPortIsOpen() {
    return sfd != SFD_UNAVAILABLE;
}



milliseconds getSteadyClockTimestampMs() {
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch());
}



ssize_t flushSerialData() {
    
    //For some reason, setting this too high can cause the serial port to not start again properly...
    float flushDurationMs = 150.0f;
    
    ssize_t result = 0;
    milliseconds startTimestampMs = getSteadyClockTimestampMs();
    while (getSteadyClockTimestampMs().count() - startTimestampMs.count() < flushDurationMs) {
        char buffer[1];
        result = readSerialData(buffer, 1);
        if (result < 0) {
            VLError("readSerialData() failed. Error: %s", strerror(errno));
        }
    };
        
    return result;
}



//Returns -1 on failure, with errno set appropriately
ssize_t writeSerialData(const char* bytesToWrite, size_t numBytesToWrite) {
    
    ssize_t numBytesWritten = write(sfd, bytesToWrite, numBytesToWrite);
    if (numBytesWritten < 0) {
        VLError("Serial port write() failed. Error: %s", strerror(errno));
        return numBytesWritten;

    }
    
    //VLLog("writeSerialData: data [%s] numBytesWritten [%d]",bytesToWrite,numBytesWritten );
    
    return numBytesWritten;
}



//Returns -1 on failure, with errno set appropriately
ssize_t readSerialData(char* const rxBuffer, size_t numBytesToReceive) {
    
    ssize_t numBytesRead = read(sfd, rxBuffer, numBytesToReceive);
    if (numBytesRead < 0) {
        VLError("Serial port read() failed. Error:%s", strerror(errno));
        return numBytesRead;

    }
    //VLLog("readSerialData: data [%s]",rxBuffer );

    return numBytesRead;
}



ssize_t closeSerialPort(void) {
    ssize_t result = 0;
    if (serialPortIsOpen()) {
        result = close(sfd);
        sfd = SFD_UNAVAILABLE;
    }
    return result;
}



int getSerialFileDescriptor(void) {
    return sfd;
}

std::string readSerialPortName() {

  

    VLLog("readConfigFile: Loading serialport [%s]", SERIAL_CONFIG_FILE);
    std::ifstream inFile(SERIAL_CONFIG_FILE);

    if (!inFile.is_open())
    {
        VLError("readConfigFile: Can't open configFile [%s]", SERIAL_CONFIG_FILE);
        return DEFAULT_PORT;
    }
    std::string lineStr;


    while (getline(inFile, lineStr))
    {
        VLLog("readConfigFile: selected serialport [%s]", lineStr.c_str());

         return lineStr;
        

    }
    return DEFAULT_PORT;

}
