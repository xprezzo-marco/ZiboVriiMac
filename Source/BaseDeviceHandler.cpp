#include <mutex>
#include <sstream>
#include <string>

#include "BaseDeviceHandler.h"
#include "BaseAircraft.h"
#include "RadioDisplay.h"
#include "VRiCommPort.h"

#include "logger.h"

#include <XPLMUtilities.h>
using namespace std;

#define MAX_COMMAND 16
static std::mutex g_aircraftMutex;

BaseDeviceHandler::BaseDeviceHandler(VRiCommPort* commPort)
	: m_commPort(commPort)
	, m_aircraft(nullptr)
	, m_dmeDistance("")
	, m_dmeSpeed("")
	, m_dmeCourse("")
	, m_dmeIdent("")
{
}

BaseDeviceHandler::~BaseDeviceHandler()
{
	if (m_aircraft != nullptr) {
		m_aircraft = nullptr;
	}

	if (m_commPort != nullptr)
	{
		delete m_commPort;
		m_commPort = nullptr;
	}
}

void BaseDeviceHandler::setAircraft(BaseAircraft* aircraft)
{
	std::lock_guard<std::mutex> guard(g_aircraftMutex);
	m_aircraft = aircraft;
}

void BaseDeviceHandler::parseCommand(string message)
{
	VriCommandParameters command = parse(message);
	bool handled = false;
	{
		std::lock_guard<std::mutex> guard(g_aircraftMutex);
		if (command.m_command != VriCommand::None && m_aircraft != nullptr)
		{
			handled = m_aircraft->handleCommand(command);
		}
	}

	
}



void BaseDeviceHandler::sendToCom(const char *command)
{
	m_commPort->send(command);
	//std::this_thread::sleep_for(std::chrono::milliseconds(20));

}




bool startsWith(const std::string& str, const std::string& cmp)
{
	return str.compare(0, cmp.length(), cmp) == 0;
}


//---------------------------------------------------------------------------
// Parse a message from the hardware by relaying it to a specific sub class
// Return the identified command, optionally with additional parameters
//---------------------------------------------------------------------------
BaseDeviceHandler::VriCommandParameters BaseDeviceHandler::parse(string message)
{
	VriCommandParameters command;
	command.m_boosted = false;
	command.m_value = 0;
	command.m_command = VriCommand::None;




	if (startsWith(message, "A")) {
		//=======================================================================
		// Handle ADF
		//=======================================================================

		if (startsWith(message, "ADFSEL1")) { command.m_command = VriCommand::AdfSel1;		return command; }
		if (startsWith(message, "ADFSEL2")) { command.m_command = VriCommand::AdfSel2;		return command; }
		if (startsWith(message, "ADFAUX")) { command.m_command = VriCommand::AdfAux;		return command; }

		if (startsWith(message, "ADF")) { 
			//ADF00000
			string value = message.substr(3, 5);

			command.m_value = stoi(value);
			command.m_command = VriCommand::Adf;
			return command;
		}

		//=======================================================================
		// Handle ALT
		//=======================================================================
		if (startsWith(message, "ALTHLD+")) { command.m_command = VriCommand::AltHldPlus;	return command; }
		if (startsWith(message, "ALTHLD-")) { command.m_command = VriCommand::AltHldMin;	return command; }
		if (startsWith(message, "ALTSEL-")) { command.m_command = VriCommand::AltSelMin;	return command; }

		if (startsWith(message, "ALT")) {

			string value = message.substr(3, 3);

			command.m_command = VriCommand::AltXXXPlus;
			command.m_float = stof(value);

			if (message.find('-') != string::npos) {
				command.m_command = VriCommand::AltXXXMin;
			}
			return command;

		}

		//=======================================================================
		// Handle APL
		//=======================================================================
		if (startsWith(message, "APLAPP+")) { command.m_command = VriCommand::AplAppPlus;		return command; }
		if (startsWith(message, "APLAPP-")) { command.m_command = VriCommand::AplAppMin;		return command; }

		if (startsWith(message, "APLAT+")) { command.m_command = VriCommand::AplAtPlus;		return command; }
		if (startsWith(message, "APLAT-")) { command.m_command = VriCommand::AplAtMin;			return command; }

		if (startsWith(message, "APLCMDA+")) { command.m_command = VriCommand::AplCmdAPlus;		return command; }
		if (startsWith(message, "APLCMDA-")) { command.m_command = VriCommand::AplCmdAMin;		return command; }

		if (startsWith(message, "APLCMDB+")) { command.m_command = VriCommand::AplCmdBPlus;		return command; }
		if (startsWith(message, "APLCMDB-")) { command.m_command = VriCommand::AplCmdBMin;		return command; }

		if (startsWith(message, "APLCMDC+")) { command.m_command = VriCommand::AplCmdCPlus;		return command; }
		if (startsWith(message, "APLCMDC-")) { command.m_command = VriCommand::AplCmdCMin;		return command; }

		if (startsWith(message, "APLCWSA+")) { command.m_command = VriCommand::AplCwsAPlus;		return command; }
		if (startsWith(message, "APLCWSA-")) { command.m_command = VriCommand::AplCwsAMin;		return command; }

		if (startsWith(message, "APLCWSB+")) { command.m_command = VriCommand::AplCwsBPlus;		return command; }
		if (startsWith(message, "APCWSB-")) { command.m_command = VriCommand::AplCwsBMin;		return command; }

		if (startsWith(message, "APLFD+")) { command.m_command = VriCommand::AplFdPlus;  		return command; }
		if (startsWith(message, "APLFD-")) { command.m_command = VriCommand::AplFdMin;			return command; }

		if (startsWith(message, "APLLOC+")) { command.m_command = VriCommand::AplLocPlus;		return command; }
		if (startsWith(message, "APLLOC-")) { command.m_command = VriCommand::AplLocMin;		return command; }

		if (startsWith(message, "APLLNAV+")) { command.m_command = VriCommand::AplLnavPlus;	return command; }
		if (startsWith(message, "APLLNAV-")) { command.m_command = VriCommand::AplLnavMin;	return command; }

		if (startsWith(message, "APLVNAV-")) { command.m_command = VriCommand::AplVnavMin;		return command; }
		if (startsWith(message, "APLVNAV+")) { command.m_command = VriCommand::AplVnavPlus;		return command; }

		if (startsWith(message, "APLTOGA+")) { command.m_command = VriCommand::AplTogaPlus;		return command; }
		if (startsWith(message, "APLTOGA-")) { command.m_command = VriCommand::AplTogaMin;		return command; }

		if (startsWith(message, "APLTOGB+")) { command.m_command = VriCommand::AplTogbPlus;		return command; }
		if (startsWith(message, "APLTOGB-+")) { command.m_command = VriCommand::AplTogbMin;		return command; }

		if (startsWith(message, "APLMAST+")) { command.m_command = VriCommand::AplMastPlus;		return command; }
		if (startsWith(message, "APLMAST-")) { command.m_command = VriCommand::AplMastMin;		return command; }

		return command;
	}

	if (startsWith(message, "B")) {

		//=======================================================================
		// handle Baro
		//=======================================================================

		if (startsWith(message, "BAR+")) { command.m_command = VriCommand::BarPlus;	return command; }
		if (startsWith(message, "BAR-")) { command.m_command = VriCommand::BarMin;	return command; }
		if (startsWith(message, "BARSEL+")) { command.m_command = VriCommand::BarSelPlus;return command; }
		if (startsWith(message, "BARSEL-")) { command.m_command = VriCommand::BarSelMin;return command; }

		return command;
	}

	if (startsWith(message, "C")) {

		//=======================================================================
		// handle ControlButtons
		//=======================================================================

		if (startsWith(message, "CTLBN0ON")) { command.m_command = VriCommand::CtlBn0On;		return command; }
		if (startsWith(message, "CTLBN1ON")) { command.m_command = VriCommand::CtlBn1On;		return command; }
		if (startsWith(message, "CTLBN2ON")) { command.m_command = VriCommand::CtlBn2On;		return command; }
		if (startsWith(message, "CTLBN3ON")) { command.m_command = VriCommand::CtlBn3On;		return command; }
		if (startsWith(message, "CTLBN4ON")) { command.m_command = VriCommand::CtlBn4On;		return command; }
		if (startsWith(message, "CTLBN5ON")) { command.m_command = VriCommand::CtlBn5On;		return command; }
		if (startsWith(message, "CTLBN6ON")) { command.m_command = VriCommand::CtlBn6On;		return command; }
		if (startsWith(message, "CTLBN7ON")) { command.m_command = VriCommand::CtlBn7On;		return command; }

		if (startsWith(message, "CTLBN0OF")) { command.m_command = VriCommand::CtlBn0Of;		return command; }
		if (startsWith(message, "CTLBN1OF")) { command.m_command = VriCommand::CtlBn1Of;		return command; }
		if (startsWith(message, "CTLBN2OF")) { command.m_command = VriCommand::CtlBn2Of;		return command; }
		if (startsWith(message, "CTLBN3OF")) { command.m_command = VriCommand::CtlBn3Of;		return command; }
		if (startsWith(message, "CTLBN4OF")) { command.m_command = VriCommand::CtlBn4Of;		return command; }
		if (startsWith(message, "CTLBN5OF")) { command.m_command = VriCommand::CtlBn5Of;		return command; }
		if (startsWith(message, "CTLBN6OF")) { command.m_command = VriCommand::CtlBn6Of;		return command; }
		if (startsWith(message, "CTLBN7OF")) { command.m_command = VriCommand::CtlBn7Of;		return command; }
		//=======================================================================
		// Handle COM... messages
		//=======================================================================
		if (startsWith(message, "COMAUX")) { command.m_command = VriCommand::ComAux;		return command; }
		if (startsWith(message, "COMSEL1")) { command.m_command = VriCommand::ComSel1;		return command; }
		if (startsWith(message, "COMSEL2")) { command.m_command = VriCommand::ComSel2;		return command; }


		if (startsWith(message, "COMs")) {

	
			string value = message.substr(4, 4);


			command.m_value = stoi(value);
			command.m_command = VriCommand::Coms;
			return command;
		}
		if (startsWith(message, "COMx")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::Comx;
			return command;
		}
		if (startsWith(message, "COMS")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::ComS;
			return command;
		}
		if (startsWith(message, "COMX")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::ComX;
			return command;
		}

		return command;
	}
	if (startsWith(message, "D")) {

		//=======================================================================
		// Handle DME
		//=======================================================================

		if (startsWith(message, "DMESEL1")) { command.m_command = VriCommand::DmeSel1;		return command; }
		if (startsWith(message, "DMESEL2")) { command.m_command = VriCommand::DmeSel2;		return command; }
		return command;
	}
	if (startsWith(message, "E")) {
		//=======================================================================
		// Handle EFI
		//=======================================================================
		if (startsWith(message, "EFIADF1")) { command.m_command = VriCommand::EfiAdf1;		return command; }
		if (startsWith(message, "EFIADF2")) { command.m_command = VriCommand::EfiAdf2;		return command; }
		if (startsWith(message, "EFIARPT")) { command.m_command = VriCommand::EfiArpt;		return command; }
		if (startsWith(message, "EFIDATA")) { command.m_command = VriCommand::EfiData;		return command; }
		if (startsWith(message, "EFIFPV")) { command.m_command = VriCommand::EfiFpv;		return command; }
		if (startsWith(message, "EFIMTR")) { command.m_command = VriCommand::EfiMtr;		return command; }
		if (startsWith(message, "EFIPOS")) { command.m_command = VriCommand::EfiPos;		return command; }
		if (startsWith(message, "EFISTA")) { command.m_command = VriCommand::EfiSta;		return command; }
		if (startsWith(message, "EFITERR")) { command.m_command = VriCommand::EfiTerr;		return command; }
		if (startsWith(message, "EFIVOR1")) { command.m_command = VriCommand::EfiVor1;		return command; }
		if (startsWith(message, "EFIVOR2")) { command.m_command = VriCommand::EfiVor2;		return command; }
		if (startsWith(message, "EFIWPT")) { command.m_command = VriCommand::EfiWpt;		return command; }
		if (startsWith(message, "EFIWX")) { command.m_command = VriCommand::EfiWx;		return command; }
		return command;
	}
	if (startsWith(message, "H")) {
		//=======================================================================
		// Handle HDG
		//=======================================================================
		if (startsWith(message, "HDGHDG+")) { command.m_command = VriCommand::HdgHdgPlus;	return command; }
		if (startsWith(message, "HDGHDG-")) { command.m_command = VriCommand::HdgHdgMin;	return command; }
		if (startsWith(message, "HDGHLD+")) { command.m_command = VriCommand::HdgHldPlus;	return command; }
		if (startsWith(message, "HDGHLD-")) { command.m_command = VriCommand::HdgHldMin;	return command; }
		if (startsWith(message, "HDGSEL+")) { command.m_command = VriCommand::HdgSelPlus;	return command; }
		if (startsWith(message, "HDGSEL-")) { command.m_command = VriCommand::HdgSelMin;		return command; }

		if (startsWith(message, "HDG")) {

			string value = message.substr(3, 3);

			command.m_command = VriCommand::HdgXXXPlus;
			command.m_float = stof(value);

			if (message.find('-') != string::npos)
			{
				command.m_command = VriCommand::HdgXXXMin;
			}
			return command;

		}
		return command;
	}
	if (startsWith(message, "M")) {
		//=======================================================================
		// handle Min
		//=======================================================================
		if (startsWith(message, "MIN+")) { command.m_command = VriCommand::MinPlus;		return command; }
		if (startsWith(message, "MIN-")) { command.m_command = VriCommand::MinMin;		return command; }
		if (startsWith(message, "MINSEL+")) { command.m_command = VriCommand::MinSelPlus;	return command; }
		if (startsWith(message, "MINSEL-")) { command.m_command = VriCommand::MinSelMin;	return command; }
		return command;
	}
	if (startsWith(message, "N")) {
		//=======================================================================
		// Handle NAV... messages
		//=======================================================================
		if (startsWith(message, "NAVAUX")) { command.m_command = VriCommand::NavAux;		return command; }
		if (startsWith(message, "NAVSEL1")) { command.m_command = VriCommand::NavSel1;		return command; }
		if (startsWith(message, "NAVSEL2")) { command.m_command = VriCommand::NavSel2;		return command; }


		if (startsWith(message, "NAVs")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::Navs;
			return command;
		}
		if (startsWith(message, "NAVx")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::Navx;
			return command;
		}
		if (startsWith(message, "NAVS")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::NavS;
			return command;
		}
		if (startsWith(message, "NAVX")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::NavX;
			return command;
		}

		//=======================================================================
		// handle NDM
		//=======================================================================
		if (startsWith(message, "NDM+")) { command.m_command = VriCommand::NdmPlus;			return command; }
		if (startsWith(message, "NDM-")) { command.m_command = VriCommand::NdmMin;		return command; }
		if (startsWith(message, "NDMSEL+")) { command.m_command = VriCommand::NdmSelPlus;	return command; }
		if (startsWith(message, "NDMSEL-")) { command.m_command = VriCommand::NdmSelMin;	return command; }

		//=======================================================================
		// handle NDR
		//=======================================================================
		if (startsWith(message, "NDR+")) { command.m_command = VriCommand::NdrPlus;			return command; }
		if (startsWith(message, "NDR-")) { command.m_command = VriCommand::NdrMin;			return command; }
		if (startsWith(message, "NDRSEL+")) { command.m_command = VriCommand::NdrSelPlus;	return command; }
		if (startsWith(message, "NDRSEL-")) { command.m_command = VriCommand::NdrSelMin;	return command; }
		return command;
	}
	if (startsWith(message, "O")) {
		//=======================================================================
		// handle OBS
		//=======================================================================
		if (startsWith(message, "OBS+")) { command.m_command = VriCommand::OBSPlus;			return command; }
		if (startsWith(message, "OBS-")) { command.m_command = VriCommand::OBSMin;			return command; }
		if (startsWith(message, "OBSSEL+")) { command.m_command = VriCommand::OBSSelPlus;	return command; }
		if (startsWith(message, "OBSSEL-")) { command.m_command = VriCommand::OBSSelMin;	return command; }
		return command;
	}
	if (startsWith(message, "S")) {
		//=======================================================================
		// handle SPD
		//=======================================================================
		if (startsWith(message, "SPDLVL+")) { command.m_command = VriCommand::SpdLvlPlus;	return command; }
		if (startsWith(message, "SPDLVL-")) { command.m_command = VriCommand::SpdLvlMin;	return command; }
		if (startsWith(message, "SPDN1+")) { command.m_command = VriCommand::SpdN1Plus;		return command; }
		if (startsWith(message, "SPDN1-")) { command.m_command = VriCommand::SpdN1Min;		return command; }
		if (startsWith(message, "SPDSPD+")) { command.m_command = VriCommand::SpdSpdMin;	return command; }
		if (startsWith(message, "SPDSPD-")) { command.m_command = VriCommand::SpdSpdPlus;	return command; }
		if (startsWith(message, "SPDSEL+")) { command.m_command = VriCommand::SpdSelPlus;	return command; }
		if (startsWith(message, "SPDSEL-")) { command.m_command = VriCommand::SpdSelMin;	return command; }

		if (startsWith(message, "SPD")) {

			string value = message.substr(3, 3);

			command.m_command = VriCommand::SpdXXXPlus;
			command.m_float = stof(value);

			if (message.find('-') != string::npos) {
				command.m_command = VriCommand::SpdXXXMin;
			}
			return command;

		}
		return command;
	}
	if (startsWith(message, "T")) {
		//=======================================================================
		// Handle TRN... messages
		//=======================================================================
		if (startsWith(message, "TRNAUX")) { command.m_command = VriCommand::TrnAux;		return command; }
		if (startsWith(message, "TRNSEL")) { command.m_command = VriCommand::TrnSel;		return command; }
		if (startsWith(message, "TRNS")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::Trns;
			return command;
		}
		if (startsWith(message, "TRNX")) {

			string value = message.substr(4, 4);

			command.m_value = stoi(value);
			command.m_command = VriCommand::Trnx;
			return command;
		}
		return command;
	}
	if (startsWith(message, "V")) {
		//=======================================================================
		// Handle VVS... messages
		//=======================================================================
		if (startsWith(message, "VVSHLD+")) { command.m_command = VriCommand::VvsHldPlus;		return command; }
		if (startsWith(message, "VVSHLD-")) { command.m_command = VriCommand::VvsHldMin;		return command; }
		if (startsWith(message, "VVS+")) { command.m_command = VriCommand::VvsPlus;			return command; }
		if (startsWith(message, "VVS-")) { command.m_command = VriCommand::VvsMin;			return command; }
	}
	return command;

}


