#ifndef VRICOMMPORTH
#define VRICOMMPORTH

#include <string.h>
#if IBM
    #include <windows.h>
#endif
#if LIN
    #include <GL/gl.h>
#elif __GNUC__
    #include <OpenGL/gl.h>
#else
    #include <GL/gl.h>
#endif
#include <thread>
#include <queue>
#include <mutex>
using namespace std;

class BaseDeviceHandler;

typedef std::queue<char*> CommandOutQueue;

class VRiCommPort
{
public:
	VRiCommPort(string szPortName);
    VRiCommPort();
    void InitPort(string szPortName);

	~VRiCommPort();

	enum Status { Init, Failed, Scanning, TimedOut, Found };

	BaseDeviceHandler *parser() const;
    
    void setParser(BaseDeviceHandler baseDeviceHandler);

    
	Status status() const;
	const char *portName() const;

	void send(const char* command);

private:
	void process();
    bool Write(char* command, int delay);
    void FMERInit();
    void MCP2AInit();
    void MCP2BInit();
    void MCP2DisplayTop(char* message);
    void MCP2DisplayBottom(char* message);

	CommandOutQueue m_commands;
	std::mutex m_commandMutex;

	char m_szPortName[64];
    
   
	std::thread *m_pollThread;

	Status m_status;

    bool m_process;

	BaseDeviceHandler *m_parser;
};



#endif
