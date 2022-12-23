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
    
    VLTrace2("sync: Radio [%s] m_frequency [%d] m_current_frequency [%d]", m_name.c_str(), m_frequency, m_current_frequency);

    updateDisplay();


    m_current_frequency = m_frequency;

}
bool RadioDisplay::updateVri2Xp(int vriValue)
{
    int freqToUpdate = vriValue;

    VLTrace2("updateVri2Xp: Radio [%s] value %d", m_name.c_str(), vriValue);


    switch (getRadioMode())
    {
    case RadioDisplay::RadioMode::Com1: //11800 tot 13700  --> COMs1820
    case RadioDisplay::RadioMode::Com1Standby:
    /*case RadioDisplay::RadioMode::Com2:*/
    case RadioDisplay::RadioMode::Com2Standby:
        freqToUpdate = freqToUpdate + 10000;

        if (freqToUpdate < 11800) {
            freqToUpdate = 11800;
        }
        else if (freqToUpdate > 13499) {
            freqToUpdate = 13499;
        }
        break;
    //case RadioDisplay::RadioMode::Nav1: //108 tot 118
    case RadioDisplay::RadioMode::Nav1Standby:
    /*case RadioDisplay::RadioMode::Nav2:*/
    case RadioDisplay::RadioMode::Nav2Standby:
        freqToUpdate = freqToUpdate + 10000;

        if (freqToUpdate < 10800) {
            freqToUpdate = 10800;
        }
        else if (freqToUpdate > 11795) {
            freqToUpdate = 11795;
        }
        break;;
    case RadioDisplay::RadioMode::Transponder:
        freqToUpdate = freqToUpdate;

        if (freqToUpdate < 0) {
            freqToUpdate = 0;
        }
        else if (freqToUpdate > 7777) {
            freqToUpdate = 7777;
        }
        break;

    case RadioDisplay::RadioMode::Adf1StandBy: // ADF17500 -> 1750
    case RadioDisplay::RadioMode::Adf2StandBy:

        if (freqToUpdate < 1900) {
            freqToUpdate = 1900;
            VLTrace2("updateVri2Xp: Radio [%s] freqToUpdate %d ", m_name.c_str(), freqToUpdate);

        }
        else if (freqToUpdate > 17500) {
            freqToUpdate = 17500;
            VLTrace2("updateVri2Xp: Radio [%s] freqToUpdate %d ", m_name.c_str(), freqToUpdate);

        }

        freqToUpdate = freqToUpdate / 10;
        VLTrace2("updateVri2Xp: Radio [%s]  freqToUpdate %d ", m_name.c_str(), freqToUpdate);


        break;

    default:
        return false;
    }
    
    VLTrace2("updateVri2Xp: Radio [%s] vriValue [%d] m_frequency [%d]", m_name.c_str(), vriValue, freqToUpdate);

    XPLMSetDatai(m_DataRef, freqToUpdate);
    return true;
    
}


bool RadioDisplay::updateXp(int value)
{
    XPLMSetDatai(m_DataRef, value);
    return true;
}
void RadioDisplay::updateDisplay() {
    switch (m_baseDeviceHandler->vriInsightEquipment) {
    case BaseDeviceHandler::VriInsightEquipment::cmdmcp2a:
        updateDisplayMCP2A();
        return;

    case BaseDeviceHandler::VriInsightEquipment::cmdmcp2b:
        updateDisplayMCP2B();
        return;
    default:
        break;

    }
    updateDisplayFMER();

}
void RadioDisplay::updateDisplayFMER()
{
    char command[MAX_DISPLAY];
    switch (getRadioMode())
    {
    case RadioDisplay::RadioMode::Com1: //11800 tot 13700  --> COMs1820
        snprintf(command, MAX_DISPLAY, "COMx%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com1Standby:
        snprintf(command, MAX_DISPLAY,"COMs%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com2:
        snprintf(command, MAX_DISPLAY, "COMX%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com2Standby:
        snprintf(command, MAX_DISPLAY, "COMS%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav1: //108 tot 118
        snprintf(command, MAX_DISPLAY, "NAVx%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav1Standby:
        snprintf(command, MAX_DISPLAY,"NAVs%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav2:
        snprintf(command, MAX_DISPLAY, "NAVX%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav2Standby:
        snprintf(command, MAX_DISPLAY, "NAVS%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Transponder:
        snprintf(command, MAX_DISPLAY, "TRN%04d0", m_frequency);
        break;

    case RadioDisplay::RadioMode::Adf1StandBy:
        snprintf(command, MAX_DISPLAY,"adf%05d", (m_frequency * 10));
        break;

    case RadioDisplay::RadioMode::Adf2StandBy:
        snprintf(command, MAX_DISPLAY, "ADF%05d", (m_frequency * 10));
        break;

    default:
        return;

    }

    m_baseDeviceHandler->sendToCom(command);

}

void RadioDisplay::updateDisplayMCP2A()
{
    char command[MAX_DISPLAY];
    switch (getRadioMode())
    {
    case RadioDisplay::RadioMode::Com1: //11800 tot 13700  --> COMs1820
        snprintf(command,MAX_DISPLAY, "COMx%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com1Standby:
        snprintf(command,MAX_DISPLAY,  "COMs%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com2:
        snprintf(command,MAX_DISPLAY, "COMX%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com2Standby:
        snprintf(command,MAX_DISPLAY,  "COMS%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav1: //108 tot 118
        snprintf(command,MAX_DISPLAY,  "NAVx%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav1Standby:
        snprintf(command,MAX_DISPLAY,  "NAVs%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav2:
        snprintf(command,MAX_DISPLAY,  "NAVX%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav2Standby:
        snprintf(command,MAX_DISPLAY,  "NAVS%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Transponder:
        snprintf(command,MAX_DISPLAY,  "TRN%04d0", m_frequency);
        break;

    case RadioDisplay::RadioMode::Adf1StandBy:
        snprintf(command,MAX_DISPLAY,  "adf%05d", (m_frequency * 10));
        break;

    case RadioDisplay::RadioMode::Adf2StandBy:
        snprintf(command,MAX_DISPLAY,  "ADF%05d", (m_frequency * 10));
        break;

    default:
        return;

    }

    m_baseDeviceHandler->sendToCom(command);

}

void RadioDisplay::updateDisplayMCP2B()
{
    char command[MAX_DISPLAY];
    switch (getRadioMode())
    {
    case RadioDisplay::RadioMode::Com1: //11800 tot 13700  --> COMs1820
        snprintf(command, MAX_DISPLAY, "COMx%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com1Standby:
        snprintf(command, MAX_DISPLAY,  "COMs%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com2:
        snprintf(command, MAX_DISPLAY,  "COMX%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Com2Standby:
        snprintf(command, MAX_DISPLAY, "COMS%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav1: //108 tot 118
        snprintf(command, MAX_DISPLAY, "NAVx%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav1Standby:
        snprintf(command, MAX_DISPLAY,  "NAVs%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav2:
        snprintf(command, MAX_DISPLAY, "NAVX%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Nav2Standby:
        snprintf(command, MAX_DISPLAY,  "NAVS%04d", (m_frequency % 10000));
        break;
    case RadioDisplay::RadioMode::Transponder:
        snprintf(command, MAX_DISPLAY,  "TRN%04d0", m_frequency);
        break;

    case RadioDisplay::RadioMode::Adf1StandBy:
        snprintf(command, MAX_DISPLAY,  "adf%05d", (m_frequency * 10));
        break;

    case RadioDisplay::RadioMode::Adf2StandBy:
        snprintf(command, MAX_DISPLAY,  "ADF%05d", (m_frequency * 10));
        break;

    default:
        return;

    }

    m_baseDeviceHandler->sendToCom(command);

}

