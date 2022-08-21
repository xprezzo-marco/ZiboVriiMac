#ifndef MCPDISPLAYH
#define MCPDISPLAYH
#include <list>

#include <XPLMUtilities.h>
#include <XPLMDataAccess.h>
#include "BaseDeviceHandler.h"

class VRiCommPort;

#define MAX_DISPLAY 9
class McpDisplay {

public:
	enum  McpMode
	{
		None=0,	Speed, Heading, Altitude
	};

	McpDisplay(BaseDeviceHandler* baseDeviceHandler,XPLMDataRef dref, McpMode radioMode);

	virtual ~McpDisplay();

	

	void sync();
	bool updateVri2Xp(float value);
	bool updateXp(float value);
	void updateDisplay();


	McpMode getMcpMode();
	float getXpValue();
	float getActualXpValue();
protected:

	

private:

	float MapVriToXp(float vriValue);

	BaseDeviceHandler* m_baseDeviceHandler;
	McpMode m_mcpMode = McpDisplay::McpMode::None;
	XPLMDataRef m_DataRef;

	
	float m_xp_value = 0;
	float m_current_xp_value = 0;



};



#endif
