#ifndef VORTEXLOGGERH
#define VORTEXLOGGERH

#include <stdarg.h>



#define PLUGIN_NAME(...) vlogger_setPluginName( __VA_ARGS__)
#define VLLog(...) vlogger_log(__VA_ARGS__)
#define VLError(...) vlogger_error(__VA_ARGS__)
#define VLTrace(...) vlogger_trace(__VA_ARGS__)
#define VLTrace2(...) vlogger_trace2(__VA_ARGS__)


void vlogger_toggleTrace1();
void vlogger_toggleTrace2();

void vlogger_setPluginName(const char *plugin);
void vlogger_log(const char* fmt, ...);
void vlogger_error(const char* fmt, ...);
void vlogger_trace(const char* fmt, ...);
void vlogger_trace2(const char* fmt, ...);

#endif
