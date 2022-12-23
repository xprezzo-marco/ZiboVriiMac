#include "logger.h"
#include <XPLMUtilities.h>
#include <XPLMProcessing.h>
#include <stdio.h>
#include <cmath>

#define MAX_PLUGIN 32
#define MAX_BUFFER 512
#define MAX_MESSAGE 1024


static char gPluginName[MAX_PLUGIN];
static char buffer[MAX_BUFFER];
static char message[MAX_MESSAGE];

bool trace1 = false;
bool trace2 = false;



void vlogger_toggleTrace1()
{
    trace1 = !trace1;
}

void vlogger_toggleTrace2()
{
    trace2 = !trace2;
}

void vlogger_setPluginName(const char *plugin)
{
    snprintf(gPluginName,MAX_PLUGIN, "%s", plugin);
}

void VLTrace(const char* fmt, ...)
{
    if (!trace1) {
        return;
    }

    va_list va;
    va_start(va, fmt);
    vsnprintf(buffer,MAX_BUFFER, fmt, va);
    va_end(va);

    float et = XPLMGetElapsedTime();

    snprintf(message, MAX_MESSAGE,"%s %1u:%02u:%06.3f Trace %s\n", gPluginName,
        (unsigned int)(et / 3600), ((unsigned int)(et / 60) % 60), std::fmod(et, 60.0),
         buffer);
    XPLMDebugString(message);
}

void VLTrace2(const char* fmt, ...)
{
    if (!trace2) {
        return;
    }



    va_list va;
    va_start(va, fmt);
    vsnprintf(buffer,MAX_BUFFER, fmt, va);
    va_end(va);

    float et = XPLMGetElapsedTime();

    snprintf(message, MAX_MESSAGE,"%s %1u:%02u:%06.3f Trace2 %s\n", gPluginName,
        (unsigned int)(et / 3600), ((unsigned int)(et / 60) % 60), std::fmod(et, 60.0),
         buffer);

    
    XPLMDebugString(message);
}

void VLLog(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    vsnprintf(buffer,MAX_BUFFER, fmt, va);
    va_end(va);

    float et = XPLMGetElapsedTime();

    snprintf(message, MAX_MESSAGE,
             "%s %1u:%02u:%06.3f Info %s\n", gPluginName,
             (unsigned int)(et / 3600), ((unsigned int)(et / 60) % 60), std::fmod(et, 60.0),
        buffer);
    XPLMDebugString(message);
}
void VLError(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    vsnprintf(buffer,MAX_BUFFER, fmt, va);
    va_end(va);

    float et = XPLMGetElapsedTime();

    snprintf(message, MAX_MESSAGE,
             "%s %1u:%02u:%06.3f Error %s\n", gPluginName,
             (unsigned int)(et / 3600), ((unsigned int)(et / 60) % 60), std::fmod(et, 60.0),
        buffer);
    XPLMDebugString(message);
}
