#include <string.h>

#include <stdio.h>
#include <list>
#include <sstream>

#include "logger.h"

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMMenus.h>


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

int g_menu_container_idx; // The index of our menu item in the Plugins menu
XPLMMenuID g_menu_id; // The menu container we'll append all our menu items to
void menu_handler(void*, void*);

XPLMCommandRef ToggleSpeechCommand = NULL;
XPLMCommandRef ToggleYokePriorityCommand = NULL;
XPLMCommandRef ToggleEfisControlCommand = NULL;
XPLMCommandRef ToggleCockpitLightsCommand = NULL;
XPLMCommandRef ToggleComboCommand = NULL;
XPLMCommandRef ToggleBaroStdCommand = NULL;
XPLMCommandRef ToggleMinimumsCommand = NULL;

int inBefore = 0;
int ToggleSpeechCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
int ToggleYokePriorityCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
int ToggleEfisControlCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon);
int ToggleCockpitLightsCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon);
int ToggleComboCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon);
int ToggleBaroStdCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon);
int ToggleMinimumsCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon);


PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{

    strcpy(outName, "XPlane 12 VrInsight Interface 3.4 MAC");
	strcpy(outSig, "zibo.xvrii.interface");
    strcpy(outDesc, "VRInsight MCP Combo panel (by xprezzo-marco)");

    vlogger_setPluginName("zibo.xvrii");
    
    g_menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "VRInsight MCP Combo", 0, 0);
        g_menu_id = XPLMCreateMenu("VRInsight MCP Combo", XPLMFindPluginsMenu(), g_menu_container_idx, menu_handler, NULL);
        XPLMAppendMenuItem(g_menu_id, "Toggle Speech", (void*)"ToggleSpeechCommand", 1);
        XPLMAppendMenuItem(g_menu_id, "Toggle Between combo modes (FMER, MCP2A, MCP2B)", (void*)"ToggleComboCommand", 1);
        XPLMAppendMenuItem(g_menu_id, "Yoke Priority. Toggle between capt/fo/both", (void*)"ToggleYokePriorityCommand", 1);
        XPLMAppendMenuItem(g_menu_id, "EFIS control. Toggle  between capt/fo", (void*)"ToggleEfisControlCommand", 1);
        XPLMAppendMenuItem(g_menu_id, "BARO mode. Toggle between BaroStd And OFF", (void*)"ToggleBaroStdCommand", 1);
        XPLMAppendMenuItem(g_menu_id, "MINS Mode. Toggle between 0 And 1", (void*)"ToggleMinimumsCommandHandler", 1);
        XPLMAppendMenuItem(g_menu_id, "Cockpit interior lights. Toggle between different modes", (void*)"ToggleCockpitLightsCommand", 1);
        XPLMAppendMenuItem(g_menu_id, "Toggle Debug Trace1", (void*)"ToggleDebugTrace1", 1);
        XPLMAppendMenuItem(g_menu_id, "Toggle Debug Trace2", (void*)"ToggleDebugTrace2", 1);

        ToggleSpeechCommand = XPLMCreateCommand("VrInsight/Command/SpeechToggle", "VRInsight MCP Combo Toggle Speech");
        ToggleYokePriorityCommand = XPLMCreateCommand("VrInsight/Command/SetYokePriority", "VRInsight MCP Combo Toggle Yoke Priority capt/fo/both");
        ToggleEfisControlCommand = XPLMCreateCommand("VrInsight/Command/SetEfisControl", "VRInsight MCP Combo Toggle EFIS control capt/fo");
        ToggleCockpitLightsCommand = XPLMCreateCommand("VrInsight/Command/SetCockpitInteriorLights", "VRInsight MCP Combo Toggle cockpit interior lights");
        ToggleComboCommand = XPLMCreateCommand("VrInsight/Command/ToggleCombo", "VRInsight MCP Combo Toggle between Combo 1 and Combo 2");
        ToggleBaroStdCommand = XPLMCreateCommand("VrInsight/Command/ToggleBaro", "VRInsight MCP Combo Toggle between Baro STD and Off");
        ToggleMinimumsCommand = XPLMCreateCommand("VrInsight/Command/ToggleMinimums", "VRInsight MCP Combo Toggle MINS Mode");



        //                            Command name,                Handler,                            Receive input before plugin windows, nRefcon
        XPLMRegisterCommandHandler(    ToggleSpeechCommand,        ToggleSpeechCommandHandler,            inBefore,        (void*) 0);
        XPLMRegisterCommandHandler(    ToggleYokePriorityCommand,    ToggleYokePriorityCommandHandler,    inBefore,        (void*)    0);
        XPLMRegisterCommandHandler(    ToggleEfisControlCommand,    ToggleEfisControlCommandHandler,    inBefore,        (void*) 0);
        XPLMRegisterCommandHandler(    ToggleCockpitLightsCommand, ToggleCockpitLightsCommandHandler,    inBefore,        (void*) 0);
        XPLMRegisterCommandHandler(ToggleBaroStdCommand, ToggleBaroStdCommandHandler,                inBefore,        (void*)0);
        XPLMRegisterCommandHandler(ToggleMinimumsCommand, ToggleMinimumsCommandHandler,                inBefore,        (void*)0);


        VLLog("Started");
        VLLog(outName);

	
	return 1;
}
void menu_handler(void* in_menu_ref, void* in_item_ref)
{


    if (!strcmp((const char*)in_item_ref, "ToggleSpeechCommand"))
    {
        if (g_plane != nullptr) { g_plane->handleSpeech(); }
        return;
    }

    if (!strcmp((const char*)in_item_ref, "ToggleYokePriorityCommand"))
    {
        if (g_plane != nullptr) { g_plane->handleYokePriority(); }
        return;
    }

    if (!strcmp((const char*)in_item_ref, "ToggleEfisControlCommand"))
    {
        if (g_plane != nullptr) { g_plane->handleEfisControl(); }
        return;
    }
    if (!strcmp((const char*)in_item_ref, "ToggleComboCommand"))
    {
        if (g_vriDeviceHandler != nullptr) { g_vriDeviceHandler->selectComboMode(); }
        return;
    }
    

    if (!strcmp((const char*)in_item_ref, "ToggleCockpitLightsCommand"))
    {
        if (g_plane != nullptr) { g_plane->handleCockpitLights(); }
        return;
    }

    if (!strcmp((const char*)in_item_ref, "ToggleBaroStdCommand"))
    {
        if (g_plane != nullptr) { g_plane->toggleBaroStdCommand(); }
        return;
    }

    if (!strcmp((const char*)in_item_ref, "ToggleMinimumsCommand"))
    {
        if (g_plane != nullptr) { g_plane->toggleMinimumsCommand(); }
        return;
    }

   

    if (!strcmp((const char*)in_item_ref, "ToggleDebugTrace1"))
    {
        vlogger_toggleTrace1();
        return;
    }

    if (!strcmp((const char*)in_item_ref, "ToggleDebugTrace2"))
    {
        vlogger_toggleTrace2();
        return;
    }
}


int ToggleSpeechCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{


    if (inPhase == xplm_CommandEnd)
    {
        if (g_plane == nullptr) { return 0; }
        g_plane->handleSpeech();
    }


    return 0;
}

int    ToggleYokePriorityCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void* inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (g_plane == nullptr) { return 0; }
        g_plane->handleYokePriority();
    }


    return 0;
}

int  ToggleEfisControlCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (g_plane == nullptr) { return 0; }
        g_plane->handleEfisControl();
    }

    return 0;
}

int ToggleBaroStdCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (g_plane == nullptr) { return 0; }
        g_plane->toggleBaroStdCommand();
    }

    return 0;
}

int ToggleMinimumsCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (g_plane == nullptr) { return 0; }
        g_plane->toggleMinimumsCommand();
    }

    return 0;
}
int  ToggleCockpitLightsCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (g_plane == nullptr) { return 0; }
        g_plane->handleCockpitLights();
    }

    return 0;
}

int  ToggleComboCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase  inPhase, void* inRefcon)
{
    if (inPhase == xplm_CommandEnd)
    {
        if (g_vriDeviceHandler == nullptr) { return 0; }

        g_vriDeviceHandler->selectComboMode();
    }

    return 0;
}

PLUGIN_API void XPluginStop(void)
{
    XPLMDestroyMenu(g_menu_id);
    
        XPLMUnregisterCommandHandler(ToggleSpeechCommand, ToggleSpeechCommandHandler, 0, 0);
        XPLMUnregisterCommandHandler(ToggleYokePriorityCommand, ToggleYokePriorityCommandHandler, 0, 0);
        XPLMUnregisterCommandHandler(ToggleEfisControlCommand, ToggleEfisControlCommandHandler, 0, 0);
        XPLMUnregisterCommandHandler(ToggleCockpitLightsCommand, ToggleCockpitLightsCommandHandler, 0, 0);
        XPLMUnregisterCommandHandler(ToggleComboCommand, ToggleComboCommandHandler, 0, 0);
        XPLMUnregisterCommandHandler(ToggleBaroStdCommand, ToggleBaroStdCommandHandler, 0, 0);
        XPLMUnregisterCommandHandler(ToggleMinimumsCommand, ToggleMinimumsCommandHandler, 0, 0);
        
	VLLog( "Stopped");

    
}

PLUGIN_API int XPluginEnable(void)
{
	VLLog( "Enabled Plugin");

	if (!g_enabled)
	{
		g_enabled = true;
        
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
        //g_plane->updateIdent("DSP0XPre");
        //g_plane->updateIdent("DSP1zzo ");


	
}
