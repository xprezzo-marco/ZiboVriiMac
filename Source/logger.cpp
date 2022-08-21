#include "logger.h"
#include <XPLMUtilities.h>
#include <XPLMProcessing.h>
#include <stdio.h>
#include <cmath>

static const char * gPluginName;
static char buffer[512];
static char message[1024];

#define VTRACE false

void vlogger_setPluginName(const char *plugin)
{
    gPluginName = plugin;
}

void vlogger_trace(const char* fmt, ...)
{
    if (!VTRACE) {
        return;
    }

    va_list va;
    va_start(va, fmt);
    vsprintf(buffer, fmt, va);
    va_end(va);

    float et = XPLMGetElapsedTime();

    sprintf(message, "%s %1u:%02u:%06.3f Trace %s\n", gPluginName,
        (unsigned int)(et / 3600), ((unsigned int)(et / 60) % 60), std::fmod(et, 60.0),
         buffer);
    XPLMDebugString(message);
}
void vlogger_log(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    vsprintf(buffer, fmt, va);
    va_end(va);

    float et = XPLMGetElapsedTime();

    sprintf(message, "%s %1u:%02u:%06.3f Info %s\n", gPluginName,
        (unsigned int)(et / 3600), ((unsigned int)(et / 60) % 60), std::fmod(et, 60.0),
        buffer);
    XPLMDebugString(message);
}
void vlogger_error(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    vsprintf(buffer, fmt, va);
    va_end(va);

    float et = XPLMGetElapsedTime();

    sprintf(message, "%s %1u:%02u:%06.3f Error %s\n", gPluginName,
        (unsigned int)(et / 3600), ((unsigned int)(et / 60) % 60), std::fmod(et, 60.0),
        buffer);
    XPLMDebugString(message);
}
