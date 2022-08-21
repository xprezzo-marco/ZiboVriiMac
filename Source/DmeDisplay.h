#ifndef DMEDISPLAYH
#define DMEDISPLAYH

#include <queue>
#include <list>

#include <XPLMUtilities.h>
#include <XPLMDataAccess.h>
#include <map>
#include "BaseDeviceHandler.h"


#define MAX_IDENT 8

class DmeDisplay {

public:
	DmeDisplay(BaseDeviceHandler* baseDeviceHandler, int nr, XPLMDataRef distRef, XPLMDataRef speedRef, XPLMDataRef courseRef, XPLMDataRef identRef);

	virtual ~DmeDisplay ();
	void sync();

	bool update(int value);
protected:

	
	XPLMDataRef m_DistRef = nullptr;
		XPLMDataRef m_SpeedRef = nullptr;
		XPLMDataRef m_CourseRef = nullptr;
		XPLMDataRef m_IdentRef = nullptr;

		int m_nr; // dme1 lower
		int m_Dist=-1;
		int m_Speed = -1;
		int m_Course = -1;
		char m_Ident[MAX_IDENT];

		char m_dmeDistance[MAX_IDENT*2] ="";
		char m_dmeSpeed[MAX_IDENT * 2] = "";
		char m_dmeCourse[MAX_IDENT * 2] = "";
		char m_dmeIdent[MAX_IDENT * 2] = "";


private:
	void updateDisplay();
	void sendIfChanged( char* newCommand, char* lastCommand, int length = 8);

	BaseDeviceHandler* m_baseDeviceHandler;

};



#endif
