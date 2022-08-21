#include "RadioDisplay.h"
#include "logger.h"

#include <list>
#include "VRiCommPort.h"

#include <XPLMUtilities.h>
#include <XPLMDataAccess.h>






RadioDisplay::RadioDisplay(BaseDeviceHandler* baseDeviceHandler, string name, XPLMDataRef dref, RadioDisplay::RadioMode radioMode) {
	m_baseDeviceHandler = baseDeviceHandler;
	m_name = name;
	m_DataRef = dref;
	m_radioMode = radioMode;

	m_frequency = getFrequency();
}


RadioDisplay::~RadioDisplay()
{
}

RadioDisplay::RadioMode RadioDisplay::getRadioMode() {
	return m_radioMode;
}

int RadioDisplay::getFrequency() {
	int frequency = XPLMGetDatai(m_DataRef);
	//VLLog("getFrequency: Radio [%s] frequency [%d]", m_name, m_frequency); 
	return frequency;
}


void RadioDisplay::sync( bool syncOnly)
{
	// we always sync based on the settings in xplane
	m_frequency = getFrequency();

	if (syncOnly) {
		return;
	}

	if (m_frequency == m_current_frequency) {
		return;
	}
	
	//VLLog((char *)"sync: Radio [%s] m_frequency [%d] m_current_frequency [%d]", m_name, m_frequency, m_current_frequency);

	updateDisplay();


	m_current_frequency = m_frequency;

}
bool RadioDisplay::updateVri2Xp(int vriValue)
{
	int freqToUpdate = vriValue;

	//VLLog("updateVri2Xp: Radio [%s] value %d", m_name, vriValue);
    RadioMode rm = getRadioMode();
    
	switch (rm)
	{
        case Com1: //11800 tot 13700  --> COMs1820
        case Com1Standby:
        case Com2Standby:
		freqToUpdate = freqToUpdate + 10000;

		if (freqToUpdate < 11800) {
			freqToUpdate = 11800;
		}
		else if (freqToUpdate > 13499) {
			freqToUpdate = 13499;
		}
		break;
	//case RadioDisplay::RadioMode::Nav1: //108 tot 118
        case Nav1Standby:
	/*case RadioDisplay::RadioMode::Nav2:*/
        case Nav2Standby:
		freqToUpdate = freqToUpdate + 10000;

		if (freqToUpdate < 10800) {
			freqToUpdate = 10800;
		}
		else if (freqToUpdate > 11795) {
			freqToUpdate = 11795;
		}
		break;
            
        case Transponder:
		freqToUpdate = freqToUpdate;

		if (freqToUpdate < 0) {
			freqToUpdate = 0;
		}
		else if (freqToUpdate > 7777) {
			freqToUpdate = 7777;
		}
		break;

        case Adf1StandBy: // ADF17500 -> 1750
        case Adf2StandBy:

		if (freqToUpdate < 1900) {
			freqToUpdate = 1900;
			//VLLog("updateVri2Xp: Radio [%s] freqToUpdate %d ", m_name, freqToUpdate);

		}
		else if (freqToUpdate > 17500) {
			freqToUpdate = 17500;
			//VLLog("updateVri2Xp: Radio [%s] freqToUpdate %d ", m_name, freqToUpdate);

		}

		freqToUpdate = freqToUpdate / 10;
		//VLLog("updateVri2Xp: Radio [%s]  freqToUpdate %d ", m_name, freqToUpdate);


		break;

	default:
		return false;
	}
	
	//VLLog("updateVri2Xp: Radio [%s] vriValue [%d] m_frequency [%d]", m_name, vriValue, freqToUpdate);

	XPLMSetDatai(m_DataRef, freqToUpdate);
	return true;
	
}


bool RadioDisplay::updateXp(int value)
{
	XPLMSetDatai(m_DataRef, value);
	return true;
}

void RadioDisplay::updateDisplay()
{
	char command[MAX_DISPLAY];
	switch (getRadioMode())
	{
	case Com1: //11800 tot 13700  --> COMs1820
		sprintf(command, "COMx%04d", (m_frequency % 10000));
		break;
	case Com1Standby:
		sprintf(command, "COMs%04d", (m_frequency % 10000));
		break;
	case Com2:
		sprintf(command, "COMX%04d", (m_frequency % 10000));
		break;
	case Com2Standby:
		sprintf(command, "COMS%04d", (m_frequency % 10000));
		break;
	case Nav1: //108 tot 118
		sprintf(command, "NAVx%04d", (m_frequency % 10000));
		break;
	case Nav1Standby:
		sprintf(command, "NAVs%04d", (m_frequency % 10000));
		break;
	case Nav2:
		sprintf(command, "NAVX%04d", (m_frequency % 10000));
		break;
	case Nav2Standby:
		sprintf(command, "NAVS%04d", (m_frequency % 10000));
		break;
	case Transponder:
		sprintf(command, "TRN%04d0", m_frequency);
		break;

	case Adf1StandBy:
		sprintf(command, "adf%05d", (m_frequency * 10));
		break;

	case Adf2StandBy:
		sprintf(command, "ADF%05d", (m_frequency * 10));
		break;

	default:
		return;

	}

	m_baseDeviceHandler->sendToCom(command);

}


