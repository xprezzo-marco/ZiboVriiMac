#include "VRiCommPort.h"
#include <string>

#include <sstream>

#include "logger.h"
#include <XPLMUtilities.h>

#include "BaseDeviceHandler.h"
#include "SerialPort.hpp"
using namespace std;
class BaseDeviceHandler;

#define MAX_COMMAND_BYTE 8
#define MAX_LINE 32

char* strPortName;

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
        m_process = true;

    m_pollThread = new thread(&VRiCommPort::process, this);
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
	m_process = false;

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
bool VRiCommPort::Write(char* command, int delay) {

    send(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    return true;
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

void VRiCommPort::process()
{
    char userInputCmd[MAX_COMMAND_BYTE] = { 0 };
    int sleepCount = 0;

    VLLog("process: started");

	std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
	try {
		while (m_process)
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
                    
                    char combo[MAX_COMMAND_BYTE + 1] = { 0 };
                    strncpy(combo, command, MAX_COMMAND_BYTE);
                    
                    VLTrace2("process: send [%s] to COMBO", combo);

					free(command);
					didSend = true;
					std::this_thread::sleep_for(std::chrono::milliseconds(30));

				}
				else
				{
					m_commandMutex.unlock();
				}
			} while (didSend);

			// Then see if there is any data coming in from the hardware, reading MAX of 32 bytes
            ssize_t readCount = readSerialData(userInputCmd, 32);
            if(readCount < 0){
				m_status = Status::Failed;
                m_process = false;
                VLLog("process: Read Error on Port");
				break;
			}

            switch (readCount) {
                        case 0:

                            // No data received, if we are in the scan phase check timeout
                            if (m_status == Status::Scanning)
                            {
                                VLTrace2("process: Status::Scanning");

                                std::chrono::duration<double> scanTime = std::chrono::system_clock::now() - start;
                                if (scanTime.count() > 30.0)
                                {
                                    VLError("process: Status::TimedOut No response from this device");

                                    m_status = Status::TimedOut;
                                    m_process = false;
                                    continue;
                                }
                            }

                            // Not scanning or no time out yet, just wait and try again
                            std::this_thread::sleep_for(std::chrono::milliseconds(5));
                            continue;

                        case MAX_COMMAND_BYTE:

                            char command[MAX_COMMAND_BYTE + 1] = { 0 };
                            strncpy(command, userInputCmd, MAX_COMMAND_BYTE);
                            command[MAX_COMMAND_BYTE] = 0;

                            VLTrace2("process: User Input [%s] on port=%s", command, m_szPortName);

                            switch (m_status) {
                            case Status::Scanning:

                                VLLog("process: Status::Scanning Input [%s] on port=%s", command, m_szPortName);

                                if (!strncmp(command, "CMDCON", 6))
                                {
                                    VLLog("process: Detected CMDCON now let's ask which ...");
                                    // It is VrInsight HW, now let's ask which...

                                    if (!Write("CMDFUN\0\0", 30)) {
                                        m_status = Status::TimedOut;
                                        m_process = false;
                                        continue;
                                    }


                                    continue;
                                }

                                if (!strncmp(command, "CMDFMER", 7)) {

                                    VLLog("process: Detected CMDFMER => its a Combo1");

                                    FMERInit();

                                    m_parser = new BaseDeviceHandler(this);
                                    m_parser->vriInsightEquipment = BaseDeviceHandler::VriInsightEquipment::cmdfmer;

                                    m_status = Status::Found;
                                    continue;

                                }

                                if (!strncmp(command, "CMDMCP2A", 8)) {

                                    VLLog("process: Detected CMDMCP2A => its a Combo2");


                                    Write("CMDVER\0\0", 30);

                                    MCP2AInit();


                                    m_parser = new BaseDeviceHandler(this);
                                    m_parser->vriInsightEquipment = BaseDeviceHandler::VriInsightEquipment::cmdmcp2a;
                                    m_status = Status::Found;

                                }
                                if (!strncmp(command, "CMDMCP2B", 8)) {

                                    VLLog("process: Detected CMDMCP2B => its a Combo2");

                                    Write("CMDVER\0\0", 30);

                                    MCP2BInit();


                                    m_parser = new BaseDeviceHandler(this);
                                    m_parser->vriInsightEquipment = BaseDeviceHandler::VriInsightEquipment::cmdmcp2b;
                                    m_status = Status::Found;

                                }

                                continue;

                            case Status::Found:

                                // Not scanning, just parse the command
                                m_parser->parseCommand(command);
                                continue;

                            default:
                                break;

                            }


                        }

                        // At 115k2 8n1... 115200/9/8 = Max 1600 bytes/second,
                        // Sleeping for 5ms allows reading of 1000/5*32 = 6400 bytes (minus processing above)
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));

                        if (sleepCount++ > 1000) {
                            VLTrace2("poll: waiting for next input");
                            sleepCount = 0;
                        }
                    }
                }
                catch (int e) {
                    VLError("poll: error %d", e);
                }
            }

            void VRiCommPort::FMERInit() {


                Write("DSP0ZIBO", 30);
                Write("DSP1 3.3", 30);
            }
                void VRiCommPort::MCP2AInit() {

                    VLLog("MCP2AInit show radio panel with text: [Powered by Xprezzo 3.3beta]");
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                    Write("dsp0 Pow", 30); // display 3 radio panel
                    Write("dsp1ered", 30);
                    Write("dsp2 by ", 30);
                    Write("dsp4 Xpr", 30);
                    Write("dsp5ezzo", 30);
                    Write("dsp6 3.3", 30);
                    Write("dsp7beta", 30);

                    VLLog("MCP2AInit show top bar display 1 and display 2 with text: [ZIBO init in progress]");
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));


                    Write("DSP0ZIBO", 30); // top bar display 1 & 2
                    Write("DSP1 ini", 30);
                    Write("DSP2t in", 30);
                    Write("DSP3 pro", 30);
                    Write("DSP4ress", 30);
                    Write("DSP5    ", 30);
                    Write("DSP6    ", 30);
                    Write("DSP7    ", 30);

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                    VLLog("MCP2AInit clear radio upper part");

                    Write("dsp0    ", 30); // clear radio
                    Write("dsp1    ", 30);
                    Write("dsp2    ", 30);
                    Write("dsp3    ", 30);

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                    VLLog("MCP2AInit clear radio lower part");

                    Write("dsp4    ", 30); // clear bottom part
                    Write("dsp5    ", 30);
                    Write("dsp6    ", 30);
                    Write("dsp7    ", 30);

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                    VLLog("MCP2AInit top bar display 1 & 2 [SPD HDG ALT VVS]");



                    Write("DSP0SPD ", 30);// top bar display 1 & 2
                    Write("DSP1    ", 30);
                    Write("DSP2HDG ", 30);
                    Write("DSP3    ", 30);
                    Write("DSP5    ", 30);
                    Write("DSP4    ", 30);
                    Write("DSP5ALT ", 30);
                    Write("DSP6    ", 30);
                    Write("DSP7VVS ", 30);

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                    VLLog("MCP2AInit clear lower bar  display 1 & 2");

                    Write("DSP8    ", 30); // bottom bar display 1 & 2
                    Write("DSP9    ", 30);
                    Write("DSPA    ", 30);
                    Write("DSPB    ", 30);
                    Write("DSPC    ", 30);
                    Write("DSPD    ", 30);
                    Write("DSPE    ", 30);
                    Write("DSPF    ", 30);
                    
                    VLLog("MCP2AInit test COM1 display");

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    Write("COMi    ", 30);
                    Write("COMx1111", 30);
                    Write("COMs2222", 30);
                    Write("dsp1*\0\0\0", 30);
                    Write("RADMhz ", 30);

                    VLLog("MCP2AInit test COM2 display");

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    Write("COMI    ", 30);
                    Write("COMX3333", 30);
                    Write("COMS4444", 30);
                    Write("dsp1*\0\0\0", 30);
                    Write("RADMhz ", 30);

                    VLLog("MCP2AInit test NAV1 display");

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    Write("NAVi    ", 30);
                    Write("NAVx5555", 30);
                    Write("NAVs6666", 30);
                    Write("dsp1*\0\0\0", 30);
                    Write("RADMhz ", 30);

                    VLLog("MCP2AInit test NAV2 display");

                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    Write("NAVI    ", 30);
                    Write("NAVX7777", 30);
                    Write("NAVX8888", 30);
                    Write("dsp1*\0\0\0", 30);
                    Write("RADMhz ", 30);

                    VLLog("MCP2AInit end of testing");

                }

                void VRiCommPort::MCP2BInit() {


                    Write("DSP0    ", 30);
                    Write("DSP1    ", 30);
                    Write("DSP8    ", 30);
                    Write("DSP9    ", 30);
                    Write("DSP2    ", 30);
                    Write("DSP3    ", 30);
                    Write("DSPA    ", 30);
                    Write("DSPB    ", 30);
                    Write("DSP4    ", 30);
                    Write("DSP5    ", 30);
                    Write("DSPC    ", 30);
                    Write("DSPD    ", 30);
                    Write("DSP6    ", 30);
                    Write("DSP7    ", 30);
                    Write("DSPE    ", 30);
                    Write("DSPF    ", 30);
                }


