#include <string.h>

#include <stdio.h>
#include <list>
#include <sstream>

#include "logger.h"

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>

#include "VRiCommPort.h"
#include "BaseAircraft.h"

#include <iostream>
#include <filesystem>
#include <vector>


using std::string;

void scanForVRiDevices();
void initPlane();

static std::list<VRiCommPort*> g_allPorts;
static std::list<BaseDeviceHandler*> g_vriDeviceHandlers;

static BaseDeviceHandler* g_vriDeviceHandler;
static VRiCommPort *commPort;
static bool g_enabled = false;


static BaseAircraft *g_plane = nullptr;

extern "C" 
{
	float mainloopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon);
}

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{

	strcpy(outName, "X Plane 11 VrInsight Interface 1.6 MAC");
	strcpy(outSig, "zibo.xvrii.interface");
	strcpy(outDesc, "Plug-in to interface directly with the VRInsight MCP Combo panel (by xprezzo-marco)");

    PLUGIN_NAME("zibo.xvrii");
	VLLog( "Started");
    XPLMSpeakString("Master Control Panel MCP plugin started");

	
	return 1;
}

PLUGIN_API void XPluginStop(void)
{
	VLLog( "Stopped");
    XPLMSpeakString("stopped");

    
}

PLUGIN_API int XPluginEnable(void)
{
	VLLog( "Enabled Plugin");
    XPLMSpeakString("enabled");

	if (!g_enabled)
	{
		g_enabled = true;
        
        XPLMSpeakString("scanning");


		// Find hardware
		scanForVRiDevices();

		if (g_vriDeviceHandler == nullptr) {
            VLLog( "g_vriDeviceHandler == nullptr");

			g_enabled = false;
			return 0;
		}

		initPlane();

		// Start the main loop
		XPLMRegisterFlightLoopCallback(mainloopCallback, 1.0, nullptr);
	}

	return 1;
}

PLUGIN_API void XPluginDisable(void)
{
	if (g_enabled)
	{
		XPLMUnregisterFlightLoopCallback(mainloopCallback, nullptr);

		// close commports and kill poll threads
		delete g_vriDeviceHandler;

		g_vriDeviceHandlers.clear();

		if (g_plane != nullptr)
		{
			delete g_plane;
			g_plane = nullptr;
		}

		g_enabled = false;
	}
	VLLog( "Disabled Plugin");
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam)
{

	if (inFrom == XPLM_PLUGIN_XPLANE && inMsg == XPLM_MSG_LIVERY_LOADED && inParam == 0)
	{
        VLLog( "XPluginReceiveMessage 2");

					// Find out which is the current user plane
			bool isSupported = g_plane->scanForPlane();

			if (!isSupported) {

				VLError("Not a supported plane yet");
				XPluginDisable();
				return;
			}

			

			VLLog("Found a supported plane");
		
	}
}

extern "C" float mainloopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	// get the currently queued commands (taking ownership of the returned queued)
	if (g_plane != nullptr )
	{
			g_plane->updateMcpDisplays();
			g_plane->updateRadios();
			CommandQueue* commands = g_plane->queuedCommands();
			if (commands != nullptr)
			{
				do
				{
					XPLMCommandOnce(commands->front());
					commands->pop();
				} while (!commands->empty());
				delete commands;
			}
			g_plane->updateMcpDisplays();
			g_plane->updateRadios();

	}

	return -5;
}

void scanForVRiDevices()
{
    
    
    commPort = new VRiCommPort();
    
    g_vriDeviceHandler = commPort->parser();

  
	XPLMSpeakString("Master Control Panel MCP detected");
}


void initPlane()
{
    XPLMSpeakString("init plane");

		VLLog( "initPlane: ");

		g_plane = new BaseAircraft(g_vriDeviceHandler);
		g_vriDeviceHandler->setAircraft(g_plane);
        g_plane->updateIdent("DSP0XPre");
        g_plane->updateIdent("DSP1zzo ");


	
}
