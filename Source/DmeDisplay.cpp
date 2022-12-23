#include "DmeDisplay.h"
#include "logger.h"

#include <list>



DmeDisplay::DmeDisplay(BaseDeviceHandler* baseDeviceHandler, int nr,XPLMDataRef distRef, XPLMDataRef speedRef, XPLMDataRef courseRef, XPLMDataRef identRef) {

	m_baseDeviceHandler = baseDeviceHandler;
	m_nr = nr;
	m_DistRef = distRef;
	m_SpeedRef = speedRef;
	m_CourseRef = courseRef;
	m_IdentRef = identRef;

	m_Dist = XPLMGetDatai(m_DistRef);
	m_Speed = XPLMGetDatai(m_SpeedRef);
	m_Course = XPLMGetDatai(m_CourseRef);

	XPLMGetDatab(m_IdentRef, m_Ident, 0, MAX_IDENT);

	VLLog("DmeDisplay: dist[%d] speed [%d] course [%d]", m_Dist, m_Speed, m_Course);


}


DmeDisplay::~DmeDisplay()
{
}

void DmeDisplay::sync()
{
	int dist = XPLMGetDatai(m_DistRef);
	

	if (dist == m_Dist) {
		// no update needed
		return;
	}

	m_Dist = dist;
	m_Speed = XPLMGetDatai(m_SpeedRef);
	m_Course = XPLMGetDatai(m_CourseRef);

	XPLMGetDatab(m_IdentRef, m_Ident, 0, MAX_IDENT);



	updateDisplay();
	
}

void DmeDisplay::updateDisplay()
{
    char command[MAX_DME_COMMAND]{0};

	if (m_nr == 1) {

		snprintf(command, MAX_DME_COMMAND, "DMEd%04u" ,(int)(m_Dist * 10.0));
		sendIfChanged( command, m_dmeDistance);


        snprintf(command, MAX_DME_COMMAND, "DMEs%03u", m_Speed);
		sendIfChanged( command, m_dmeSpeed);

        snprintf(command, MAX_DME_COMMAND,  "DMc%03u ", (int)(m_Course + 360.5) % 360);
		sendIfChanged( command, m_dmeCourse);

		strcpy(command, "DMi");
		strncat(command, m_Ident, 5);
		strncat(command, "        ", 8);
		sendIfChanged( command, m_dmeIdent);

		return;
	}

    snprintf(command, MAX_DME_COMMAND, "DMED%04u", (int)(m_Dist * 10.0));
	sendIfChanged(command, m_dmeDistance);

    snprintf(command, MAX_DME_COMMAND,  "DMES%03u", (int)m_Speed);
	sendIfChanged( command, m_dmeSpeed);

    snprintf(command, MAX_DME_COMMAND,  "DMC%03u ", (int)(m_Course + 360.5) % 360);
	sendIfChanged( command, m_dmeCourse);

	strcpy(command, "DMI");
	strncat(command, m_Ident, 5);
	strncat(command, "        ", 8);
	sendIfChanged( command, m_dmeIdent);

}

void DmeDisplay::sendIfChanged(char* newCommand, char* lastCommand, int length)
{
	if (strncmp(lastCommand, newCommand, length) == 0)
	{
		return;
	}

	m_baseDeviceHandler->sendToCom(newCommand);

	strncpy(lastCommand, newCommand, length);
}

bool DmeDisplay::update(int value)
{
	if (m_Course != value) {
		XPLMSetDatai(m_CourseRef, value);
		return true;
	}

	return false;
}







