#ifndef BASEAIRCRAFTH
#define BASEAIRCRAFTH

#include <queue>
#include <list>
#include <string>

#include <XPLMUtilities.h>
#include <XPLMDataAccess.h>
#include <map>
#include "BaseDeviceHandler.h"
#include "RadioDisplay.h"
#include "DmeDisplay.h"
#include "McpDisplay.h"

class VRiCommPort;
class BaseDeviceHandler;

typedef std::queue<XPLMCommandRef> CommandQueue;

#define MAX_TAIL_NR 16
#define PLUGIN_CONFIG_PATH "./Resources/plugins/ZiboVrII/config/"
#define NO_IDENT0 "DSP0    "
#define NO_IDENT1 "DSP1    "



class BaseAircraft
{
public:
    BaseAircraft(BaseDeviceHandler* baseDeviceHandler);
    virtual ~BaseAircraft();

    void updateMcpDisplays();
    void updateRadios();
    void updateIdent(const char* ident);
    void initializeRadios();

    virtual bool handleCommand(BaseDeviceHandler::VriCommandParameters command);
    CommandQueue* queuedCommands();
    bool isLoaded();
    bool scanForPlane();
    char m_tailNum[MAX_TAIL_NR] = { 0 };


protected:
    bool checkFastToggle(uint64_t& last, uint64_t interval);
    bool readConfigFile(string fileName);

    map<BaseDeviceHandler::VriCommand, XPLMCommandRef> cmdMap;
    map<BaseDeviceHandler::VriDataRef, XPLMDataRef> drMap;
    BaseDeviceHandler::VriCommand ConvertToVriCommand(string target);
    BaseDeviceHandler::VriDataRef ConvertToVriDataref(string target);


    XPLMDataRef getDataRef(BaseDeviceHandler::VriDataRef vriDataRef);

    XPLMDataRef findDataRef(const char* reference);
    XPLMCommandRef findCommandRef(const char* command);




    McpDisplay* m_altitude = nullptr;
    McpDisplay* m_speed = nullptr;
    McpDisplay* m_heading = nullptr;
    



    XPLMDataRef drSpeed;
    XPLMDataRef drHeading;
    XPLMDataRef drAltitude;

    XPLMDataRef drCom1;
    XPLMDataRef drCom1Standby;
    XPLMDataRef drCom2;
    XPLMDataRef drCom2Standby;

    XPLMDataRef drNav1;
    XPLMDataRef drNav1Standby;
    XPLMDataRef drNav2;
    XPLMDataRef drNav2Standby;

    XPLMDataRef drAdf1;
    XPLMDataRef drAdf1Standby;
    XPLMDataRef drAdf2;
    XPLMDataRef drAdf2Standby;

    XPLMDataRef drTransponder;

    XPLMDataRef drDme1Dist;
    XPLMDataRef drfDme1Ident;
    XPLMDataRef drDme1Speed;
    XPLMDataRef drDme1Course;

    XPLMDataRef drDme2Dist;
    XPLMDataRef drfDme2Ident;
    XPLMDataRef drDme2Speed;
    XPLMDataRef drDme2Course;

    bool isPlaneConfigured = false;

    int m_adf1LastValue = -1;
    int m_adf2LastValue = -1;

    RadioDisplay* m_com1Act;
    RadioDisplay* m_com1Stby;

    RadioDisplay* m_com2Act;
    RadioDisplay* m_com2Stby;

    RadioDisplay* m_nav1Act;
    RadioDisplay* m_nav1Stby;

    RadioDisplay* m_nav2Act;
    RadioDisplay* m_nav2Stby;



    RadioDisplay* m_adf1Act;
    RadioDisplay* m_adf1Stby;

    RadioDisplay* m_adf2Act;
    RadioDisplay* m_adf2Stby;


    DmeDisplay* m_dme1;
    DmeDisplay* m_dme2;

    RadioDisplay* m_transponder;


    int lightMode = 0;


    bool speechActive = true;
    bool hdgUpperSelected = false;
    int bankAngle = 0;
    bool minUpperSelected = false;
    bool baroUpperSelected = false;

    uint64_t minSelectedMs = 0;
    uint64_t baroSelectedMs = 0;

   
    bool getXPLMCommandRef(BaseDeviceHandler::VriCommand vriCommand, XPLMCommandRef& result);
    bool getXPLMDataRef(BaseDeviceHandler::VriDataRef vriDataRef, XPLMDataRef& result);
    float getDataf(BaseDeviceHandler::VriDataRef vriDataRef, float defValue);
    void setDataf(BaseDeviceHandler::VriDataRef vriDataRef, float value);

    int getDatai(BaseDeviceHandler::VriDataRef vriDataRef, int defValue);
    void setDatai(BaseDeviceHandler::VriDataRef vriDataRef, int value);
    bool getDatab(BaseDeviceHandler::VriDataRef vriDataRef, void* data, int max);

    void setDatavf(BaseDeviceHandler::VriDataRef vriDataRef, float* inValues, int inoffset, int inCount);

    bool storeCommand(BaseDeviceHandler::VriCommand vriCommand, const char* reference);
    bool storeDataRef(BaseDeviceHandler::VriDataRef vriDataRef, const char* reference);
    bool initData();
    bool initDataRef();
    bool initCommand();

    bool scheduleCommand(BaseDeviceHandler::VriCommand vriCommand, int repeat = 1);
    bool handleBankAngle(BaseDeviceHandler::VriCommand command);
    int updateBankAngle();

    bool scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand vriCommand, const char* speech,int repeat = 1);
    bool scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef vriDataRef, BaseDeviceHandler::VriCommand VriCommand,
        const char* speech, const char* ident0, const char* ident1, int repeat = 1);
    bool scheduleButtonCommand(BaseDeviceHandler::VriDataRef ref, BaseDeviceHandler::VriCommand vriCommand, const char* speech, int repeat = 1);
    void Speak(const char* speech);
    bool transferRadio(const char* radio, RadioDisplay* act, RadioDisplay* stby);
    void selectRadio(const char* radio,  RadioDisplay* stby);

private:
    bool commandDefined(BaseDeviceHandler::VriCommand VriCommand);
    bool dataRefDefined(BaseDeviceHandler::VriDataRef vriDataRef);

    bool readConfigFile(const char* fileName);
    void setMapMode(BaseDeviceHandler::VriCommand vriCommand);
    void setMapRange(BaseDeviceHandler::VriCommand vriCommand);
    void setEfiAdf(BaseDeviceHandler::VriDataRef vriDataRef, BaseDeviceHandler::VriCommand vriCommand);
    void setEfiVor(BaseDeviceHandler::VriDataRef vriDataRef, BaseDeviceHandler::VriCommand vriCommand);

    void initMcpDisplays();
    void initRadios();
    void setLights();
    
    void setPanelLight(float captMainPanel, float foMainPanel, float overheadPanel, float pedestalPanel);
    void setGenericLight(float forwardPanelFlood, float glareShieldFlood, float pedestalFlood,
        float captChartLight, float foChartLight, float circuitBrakerFlood);

    void setCourse(BaseDeviceHandler::VriCommand VriCommand);
    void syncCourse();
    void setVvs(BaseDeviceHandler::VriCommand VriCommand);

    void resetVvs();
    void displayVvs();

    bool startsWith(const std::string& str, const std::string& cmp);
    bool contains(const std::string& str, char cmp);

        std::string TrimFunction(string str);

    void simulateKey(int keyNumber);

    CommandQueue* m_xpCommands;
    BaseDeviceHandler* m_baseDeviceHandler;

    RadioDisplay::RadioMode m_radioMode;
};


#endif
