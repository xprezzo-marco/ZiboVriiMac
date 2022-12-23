#ifndef RADIODISPLAYH
#define RADIODISPLAYH
#include <list>

#include <XPLMUtilities.h>
#include <XPLMDataAccess.h>
#include "BaseDeviceHandler.h"

class VRiCommPort;

#define MAX_DISPLAY 9
class RadioDisplay {

public:
    enum RadioMode
        {
            None,
            Com1, Com1Standby,
            Com2, Com2Standby,
            Nav1, Nav1Standby,
            Nav2, Nav2Standby,
            Adf1, Adf1StandBy,
            Adf2, Adf2StandBy,
            Dme1,
            Dme2,
            Transponder
        };

	RadioDisplay(BaseDeviceHandler* baseDeviceHandler, string name, XPLMDataRef dref, RadioMode radioMode);

	virtual ~RadioDisplay();

	

	void sync(bool syncOnly = false);
	bool updateVri2Xp(int value);
	bool updateXp(int value);

	RadioMode getRadioMode();
	int getFrequency();
protected:

	

private:

    void updateDisplay();
        void updateDisplayFMER();
        void updateDisplayMCP2A();
        void updateDisplayMCP2B();	int MapVriToXp(int vriValue);

	string m_name;
	RadioMode m_radioMode;
	XPLMDataRef m_DataRef;

	bool init = false;

	int m_frequency;
	int m_current_frequency = -1;

	BaseDeviceHandler* m_baseDeviceHandler;
};



#endif
