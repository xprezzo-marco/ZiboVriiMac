//
//  SerialPort.hpp
//  ZiboMacVriiPlugin
//
//  Created by Marco van Zijl on 05/08/2022.
//

#ifndef SerialPort_hpp
#define SerialPort_hpp

#include <stdio.h>
#include <string>



#include <unistd.h> //ssize_t



int openAndConfigureSerialPort(const char* portPath, int baudRate);



bool serialPortIsOpen();



ssize_t flushSerialData();



ssize_t writeSerialData(const char* bytes, size_t length);



ssize_t readSerialData(char* bytes, size_t length);



ssize_t closeSerialPort(void);



int getSerialFileDescriptor(void);

std::string readSerialPortName();



#endif //__SERIAL_PORT_HPP__

