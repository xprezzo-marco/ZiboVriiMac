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
		None =0,
		Com1 =1,
        Com1Standby = 2,
		Com2 = 3,
        Com2Standby = 4,
		Nav1 = 5,
        Nav1Standby = 6,
		Nav2= 7,
        Nav2Standby = 8,
		Adf1 = 9,
        Adf1StandBy =10,
		Adf2=11,
        Adf2StandBy = 12,
		Dme1 = 13,
		Dme2 = 14,
		Transponder = 15
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
	int MapVriToXp(int vriValue);

	string m_name;
	RadioMode m_radioMode;
	XPLMDataRef m_DataRef;

	bool init = false;

	int m_frequency;
	int m_current_frequency = -1;

	BaseDeviceHandler* m_baseDeviceHandler;
};



#endif
