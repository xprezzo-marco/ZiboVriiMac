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
	char command[9];

	if (m_nr == 1) {

		sprintf(command, "DMEd%04u", (int)(m_Dist * 10.0));
		sendIfChanged( command, m_dmeDistance);


		sprintf(command, "DMEs%03u", m_Speed);
		sendIfChanged( command, m_dmeSpeed);

		sprintf(command, "DMc%03u ", (int)(m_Course + 360.5) % 360);
		sendIfChanged( command, m_dmeCourse);

		strcpy(command, "DMi");
		strncat(command, m_Ident, 5);
		strncat(command, "        ", 8);
		sendIfChanged( command, m_dmeIdent);

		return;
	}

	sprintf(command, "DMED%04u", (int)(m_Dist * 10.0));
	sendIfChanged(command, m_dmeDistance);

	sprintf(command, "DMES%03u", (int)m_Speed);
	sendIfChanged( command, m_dmeSpeed);

	sprintf(command, "DMC%03u ", (int)(m_Course + 360.5) % 360);
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







