#include "VRiCommPort.h"
#include <string>

#include <sstream>

#include "logger.h"
#include <XPLMUtilities.h>

#include "BaseDeviceHandler.h"
#include "SerialPort.hpp"
using namespace std;
class BaseDeviceHandler;


VRiCommPort::VRiCommPort(){
    std:string szPortName =  readSerialPortName();
    
    InitPort(szPortName);

}


VRiCommPort::VRiCommPort(string szPortName)
{
    InitPort(szPortName);
}

void VRiCommPort::InitPort(string szPortName){
    
    const int baudRate = 115200;
    
    int sfd = openAndConfigureSerialPort(szPortName.c_str(), baudRate);
    if (sfd < 0) {
        if (sfd == -1) {
            VLLog("InitPort: Unable to connect to serial port.\n");
        }
        else { //sfd == -2
            VLLog("InitPort: Error setting serial port attributes.\n");
        }
        m_status = Status::Failed;

        return;
    }
	


	
        
        const char deviceInit[]="CMDRST\0\0CMDCON\0\0";
        m_poll = true;

    m_pollThread = new thread(&VRiCommPort::poll, this);
        if (m_pollThread)
        {
        
        ssize_t status = writeSerialData(deviceInit, 16);

		if (status > 0)
		{
			VLLog("%s scan ok, did send RST and CON commands [%d].", szPortName.c_str(), status);
            m_parser = new BaseDeviceHandler(this);

			m_status = Status::Found;
                     
        }else{
        
		
			VLLog("%s scan failed to write init codes" , szPortName.c_str());
			m_status = Status::Failed;
            closeSerialPort();
        }
        }else{
                VLLog("%s scan failed to start scan thread with ", szPortName.c_str());
                m_status = Status::Failed;
            }
	
}

VRiCommPort::~VRiCommPort()
{
	m_poll = false;

	if (m_pollThread != nullptr)
	{
		m_pollThread->join();
		delete m_pollThread;
	}

	if (m_parser != nullptr)
		m_parser = nullptr;

	
}
void VRiCommPort::setParser(BaseDeviceHandler baseDeviceHandler)
{
    *m_parser = baseDeviceHandler;
}
BaseDeviceHandler *VRiCommPort::parser() const
{
	return m_parser;
}

const char *VRiCommPort::portName() const
{
	return m_szPortName;
}

VRiCommPort::Status VRiCommPort::status() const
{
	return m_status;
}



void VRiCommPort::send(const char* command)
{
    
    VLLog("send: command [%s] to m_commands");
    size_t length = strlen(command);
            char *c = (char*)malloc(length);
            if (c == 0) {
                VLError("send: malloc failed");
                return;
            }

            strncpy(c, command, length);

            m_commandMutex.lock();
            m_commands.push(c);
            m_commandMutex.unlock();
    
  
}

void VRiCommPort::poll()
{
	char commandBuffer[64] = { 0 };
	int commandLength = 0;
	char newcmd[32];
    VLLog("poll: started");

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	try {
		while (m_poll)
		{
			bool didSend;

			// First send any pending commands from the plugin to the hardware
			do
			{
				didSend = false;
				m_commandMutex.lock();
                
               // VLLog("poll: m_commands.size=%d",m_commands.size());
				if (m_commands.size() > 0)
				{
					char* command = m_commands.front();
					m_commands.pop();

					// unlock asap and each iteration to not block the producing thread
					m_commandMutex.unlock();

                    writeSerialData(command, 8);
                    
					free(command);
					didSend = true;
					std::this_thread::sleep_for(std::chrono::milliseconds(100));

				}
				else
				{
					m_commandMutex.unlock();
				}
			} while (didSend);

			// Then see if there is any data coming in from the hardware, reading MAX of 32 bytes
            ssize_t readCount = readSerialData(newcmd, 32);
            if(readCount < 0){
				m_status = Status::Failed;
				m_poll = false;
				VLLog( "Read Error on Port");
				break;
			}

			if (readCount == 0)
			{
				// No data received, if we are in the scan phase check timeout
				if (m_status == Status::Scanning)
				{
					std::chrono::duration<double> scanTime = std::chrono::system_clock::now() - start;
					if (scanTime.count() > 3.0)
					{
						m_status = Status::TimedOut;
						m_poll = false;
						continue;
					}
				}

				// Not scanning or no time out yet, just wait and try again
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				continue;
			}

            //VLLog( "data received %s", newcmd);

			// replace 0's with .'s for easier string handling
			for (int i = 0; i < readCount; i++)
				if (newcmd[i] == 0x00)
					newcmd[i] = 0x2e;

			// append the new data to any remaining data from the previous loop
			commandLength += readCount;
			strncat(commandBuffer, newcmd, readCount);

			// Reducing buffer to below 8 bytes, so commandBuffer max size = 7+32 = 39 bytes
			// Aka: Keep looping while any potential command remains in the buffer
			while (commandLength >= 8)
			{
				// Get the head message (assumes all VrInsight hardware send 8 byte commands)
				char cmdNow[16];
				strncpy(cmdNow, commandBuffer, 8);
				cmdNow[8] = 0;
                VLLog("Received <%s> on <%s>", cmdNow, m_szPortName);

				
					// Not scanning, just parse the command
					m_parser->parseCommand(cmdNow);
				

				// Shift the buffer
				strncpy(commandBuffer, &commandBuffer[8], commandLength - 8);
				commandBuffer[commandLength - 8] = 0;
				commandLength -= 8;
			}

			// At 115k2 8n1... 115200/9/8 = Max 1600 bytes/second, 
			// Sleeping for 5ms allows reading of 1000/5*32 = 6400 bytes (minus processing above)
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
	}
	catch (int e) {
		VLError("poll: error %d", e);
	}
}
