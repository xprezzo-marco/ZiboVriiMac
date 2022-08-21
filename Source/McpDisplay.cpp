#include "McpDisplay.h"
#include "logger.h"

#include <list>
#include "VRiCommPort.h"

#include <XPLMUtilities.h>
#include <XPLMDataAccess.h>



McpDisplay::McpDisplay(BaseDeviceHandler* baseDeviceHandler, XPLMDataRef dref, McpDisplay::McpMode mcpMode)  {

	m_baseDeviceHandler = baseDeviceHandler;
	m_DataRef = dref;
	m_mcpMode = mcpMode;

	//VLTrace("McpDisplay: McpMode[%d] init", mcpMode);
}


McpDisplay::~McpDisplay()
{
}

McpDisplay::McpMode McpDisplay::getMcpMode() {
	return m_mcpMode;
}

float McpDisplay::getXpValue() {
	return m_xp_value;
}

float McpDisplay::getActualXpValue() {

	if (m_DataRef == nullptr) {
		VLTrace("getActualXpValue: McpMode[%d] m_DataRef == null", getMcpMode());
		return -1;
	}

	float dvalue = XPLMGetDataf(m_DataRef);

	VLTrace("getActualXpValue: McpMode[%d] value = %d", getMcpMode(), dvalue);

	return dvalue;
}



void McpDisplay::sync()
{

	VLTrace("sync: [%d]", getMcpMode());

	
	
		// we always sync based on the settings in xplane
		m_xp_value = getActualXpValue();
    VLTrace("sync: McpMode [%d] xpValue [%d] ", getMcpMode(), m_xp_value);
	
	if (m_current_xp_value == m_xp_value) {
		return;
	}
	m_current_xp_value = m_xp_value;

	updateDisplay();


}
bool McpDisplay::updateVri2Xp(float vriValue)
{

	m_xp_value = MapVriToXp(vriValue);

	XPLMSetDataf(m_DataRef, m_xp_value);

	return true;

}

float McpDisplay::MapVriToXp(float vriValue) {

	float retval = vriValue;
	switch (getMcpMode())
	{
        case Speed:
	case Heading:
		break;

	case Altitude:
		retval = vriValue * 100;
		break;

	default:
		break;
	}

	return retval;

}



bool McpDisplay::updateXp(float xpValue)
{
	if (m_xp_value != xpValue) {


		XPLMSetDataf(m_DataRef, xpValue);
		return true;
	}

	return false;
}

void McpDisplay::updateDisplay()
{
	char command[MAX_DISPLAY];
	switch (getMcpMode())
	{
	case Speed:
		sprintf(command, "SPD%03.0f", m_xp_value);
		break;
	case Heading:
		sprintf(command, "HDG%03.0f", m_xp_value);
		break;
	case Altitude:
		sprintf(command, "ALT%03.0f", (m_xp_value / 100));
		break;
	
	default:
		return;

	}

	m_baseDeviceHandler->sendToCom(command);
}



