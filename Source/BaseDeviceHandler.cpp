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
void BaseDeviceHandler::selectComboMode()
{
    switch (vriInsightEquipment) {
    case BaseDeviceHandler::VriInsightEquipment::cmdfmer:
        vriInsightEquipment = BaseDeviceHandler::VriInsightEquipment::cmdmcp2a;
        XPLMSpeakString("MCP2A combo 2 airbus Selected");
        VLLog("MCP2A combo 2 airbus Selected");
        return;

    case BaseDeviceHandler::VriInsightEquipment::cmdmcp2a:
        vriInsightEquipment = BaseDeviceHandler::VriInsightEquipment::cmdmcp2b;
        XPLMSpeakString("MCP2B combo 2 boeing Selected");
        VLLog("MCP2B combo 2 boeing Selected");

        return;

    default:
        vriInsightEquipment = BaseDeviceHandler::VriInsightEquipment::cmdfmer;
        XPLMSpeakString("FMER combo 1 Mode Selected");
        VLLog("FMER combo 1 Mode Selected");
        return;


    }


    return;
}



void BaseDeviceHandler::setAircraft(BaseAircraft* aircraft)
{
    std::lock_guard<std::mutex> guard(g_aircraftMutex);
    m_aircraft = aircraft;
}


void BaseDeviceHandler::parseCommand(string message)
{
    if (m_aircraft == nullptr)
    {
        VLTrace2("parseCommand: m_aircraft == nullptr %s ignored", message.c_str());
        return;
    }

    VriCommandParameters command = parse(message);
    if (command.m_command == VriCommand::None) {

        VLTrace2("parseCommand: command.m_command == VriCommand::None %s ignored", message.c_str());
        return;
    }

    bool handled = false;
    {
        std::lock_guard<std::mutex> guard(g_aircraftMutex);

        handled = m_aircraft->handleCommand(command);

    }


    if (!handled)
    {
        VLError("%s translated to internal command #%d and was %s by the aircraft", message.c_str(), command.m_command, "UNHANDLED");
    }
}



void BaseDeviceHandler::sendToCom(const char* command)
{
    m_commPort->send(command);
    VLTrace("sendToCom: [%s]", command);
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
BaseDeviceHandler::VriCommandParameters BaseDeviceHandler::parse(string message) {


    switch (vriInsightEquipment) {
    default:
    case BaseDeviceHandler::VriInsightEquipment::cmdfmer:
        return parseFMER(message);

    case BaseDeviceHandler::VriInsightEquipment::cmdmcp2a:
        return parseMCP2A(message);

    case BaseDeviceHandler::VriInsightEquipment::cmdmcp2b:
        return parseMCP2B(message);

    }


}

BaseDeviceHandler::VriCommandParameters BaseDeviceHandler::parseFMER(string message)
{
    VriCommandParameters command;
    command.m_boosted = false;
    command.m_value = 0;
    command.m_command = VriCommand::None;

    VLTrace("parseCombo1: message %s", message.c_str());



    // REORDERED TO GIVE PRIORITY TO BUTTONS
    if (startsWith(message, "SPDN1+")) { command.m_command = VriCommand::SpdN1Plus;        return command; }
    if (startsWith(message, "SPDN1-")) { command.m_command = VriCommand::SpdN1Min;        return command; }

    // first the prio is the SPD ALT and HDG
    if (startsWith(message, "SPD") && has_any_digits(message)) {
        string value = message.substr(3, 3);

        command.m_command = VriCommand::SpdXXXPlus;
        command.m_float = stof(value);

        if (message.find('-') != string::npos) {
            command.m_command = VriCommand::SpdXXXMin;
        }
        return command;
    }

    if (startsWith(message, "ALT") && has_any_digits(message)) {

        string value = message.substr(3, 3);

        command.m_command = VriCommand::AltXXXPlus;
        command.m_float = stof(value);

        if (message.find('-') != string::npos) {
            command.m_command = VriCommand::AltXXXMin;
        }
        return command;

    }

    if (startsWith(message, "HDG") && has_any_digits(message)) {

        string value = message.substr(3, 3);

        command.m_command = VriCommand::HdgXXXPlus;
        command.m_float = stof(value);

        if (message.find('-') != string::npos)
        {
            command.m_command = VriCommand::HdgXXXMin;
        }
        return command;

    }

    if (startsWith(message, "A")) {

        if (startsWith(message, "ALT")) {

            // more prio to the rotary
            //
            //=======================================================================
            // Handle ALT
            //=======================================================================
            if (startsWith(message, "ALTHLD+")) { command.m_command = VriCommand::AltHldPlus;    return command; }
            if (startsWith(message, "ALTHLD-")) { command.m_command = VriCommand::AltHldMin;    return command; }
            if (startsWith(message, "ALTSEL-")) { command.m_command = VriCommand::AltSelMin;    return command; }
            if (startsWith(message, "ALTSEL+")) { command.m_command = VriCommand::AltSelPlus;    return command; }


            return command;

        }

        //=======================================================================
        // Handle ADF
        //=======================================================================

        if (startsWith(message, "ADFSEL1")) { command.m_command = VriCommand::AdfSel1;        return command; }
        if (startsWith(message, "ADFSEL2")) { command.m_command = VriCommand::AdfSel2;        return command; }
        if (startsWith(message, "ADFAUX")) { command.m_command = VriCommand::AdfAux;        return command; }

        if (startsWith(message, "ADF")) {
            //ADF00000
            string value = message.substr(3, 5);

            command.m_value = stoi(value);
            command.m_command = VriCommand::Adf;
            return command;
        }



        //=======================================================================
        // Handle APL
        //=======================================================================
        if (startsWith(message, "APLAPP+")) { command.m_command = VriCommand::AplAppPlus;        return command; }
        if (startsWith(message, "APLAPP-")) { command.m_command = VriCommand::AplAppMin;        return command; }

        if (startsWith(message, "APLAT+")) { command.m_command = VriCommand::AplAtPlus;        return command; }
        if (startsWith(message, "APLAT-")) { command.m_command = VriCommand::AplAtMin;            return command; }

        if (startsWith(message, "APLCMDA+")) { command.m_command = VriCommand::AplCmdAPlus;        return command; }
        if (startsWith(message, "APLCMDA-")) { command.m_command = VriCommand::AplCmdAMin;        return command; }

        if (startsWith(message, "APLCMDB+")) { command.m_command = VriCommand::AplCmdBPlus;        return command; }
        if (startsWith(message, "APLCMDB-")) { command.m_command = VriCommand::AplCmdBMin;        return command; }

        if (startsWith(message, "APLCMDC+")) { command.m_command = VriCommand::AplCmdCPlus;        return command; }
        if (startsWith(message, "APLCMDC-")) { command.m_command = VriCommand::AplCmdCMin;        return command; }

        if (startsWith(message, "APLCWSA+")) { command.m_command = VriCommand::AplCwsAPlus;        return command; }
        if (startsWith(message, "APLCWSA-")) { command.m_command = VriCommand::AplCwsAMin;        return command; }

        if (startsWith(message, "APLCWSB+")) { command.m_command = VriCommand::AplCwsBPlus;        return command; }
        if (startsWith(message, "APLCWSB-")) { command.m_command = VriCommand::AplCwsBMin;        return command; }

        if (startsWith(message, "APLFD+")) { command.m_command = VriCommand::AplFdPlus;          return command; }
        if (startsWith(message, "APLFD-")) { command.m_command = VriCommand::AplFdMin;            return command; }

        if (startsWith(message, "APLLOC+")) { command.m_command = VriCommand::AplLocPlus;        return command; }
        if (startsWith(message, "APLLOC-")) { command.m_command = VriCommand::AplLocMin;        return command; }

        if (startsWith(message, "APLLNAV+")) { command.m_command = VriCommand::AplLnavPlus;    return command; }
        if (startsWith(message, "APLLNAV-")) { command.m_command = VriCommand::AplLnavMin;    return command; }

        if (startsWith(message, "APLVNAV-")) { command.m_command = VriCommand::AplVnavMin;        return command; }
        if (startsWith(message, "APLVNAV+")) { command.m_command = VriCommand::AplVnavPlus;        return command; }

        if (startsWith(message, "APLTOGA+")) { command.m_command = VriCommand::AplTogaPlus;        return command; }
        if (startsWith(message, "APLTOGA-")) { command.m_command = VriCommand::AplTogaMin;        return command; }

        if (startsWith(message, "APLTOGB+")) { command.m_command = VriCommand::AplTogbPlus;        return command; }
        if (startsWith(message, "APLTOGB-+")) { command.m_command = VriCommand::AplTogbMin;        return command; }

        if (startsWith(message, "APLMAST+")) { command.m_command = VriCommand::AplMastPlus;        return command; }
        if (startsWith(message, "APLMAST-")) { command.m_command = VriCommand::AplMastMin;        return command; }

        return command;
    }

    if (startsWith(message, "B")) {

        //=======================================================================
        // handle Baro
        //=======================================================================

        if (startsWith(message, "BAR+")) { command.m_command = VriCommand::BarPlus;    return command; }
        if (startsWith(message, "BAR-")) { command.m_command = VriCommand::BarMin;    return command; }
        if (startsWith(message, "BARSEL+")) { command.m_command = VriCommand::BarSelPlus;return command; }
        if (startsWith(message, "BARSEL-")) { command.m_command = VriCommand::BarSelMin;return command; }

        return command;
    }

    if (startsWith(message, "C")) {

        //=======================================================================
        // handle ControlButtons
        //=======================================================================

        if (startsWith(message, "CTLBN0ON")) { command.m_command = VriCommand::CtlBn0On;        return command; }
        if (startsWith(message, "CTLBN1ON")) { command.m_command = VriCommand::CtlBn1On;        return command; }
        if (startsWith(message, "CTLBN2ON")) { command.m_command = VriCommand::CtlBn2On;        return command; }
        if (startsWith(message, "CTLBN3ON")) { command.m_command = VriCommand::CtlBn3On;        return command; }
        if (startsWith(message, "CTLBN4ON")) { command.m_command = VriCommand::CtlBn4On;        return command; }
        if (startsWith(message, "CTLBN5ON")) { command.m_command = VriCommand::CtlBn5On;        return command; }
        if (startsWith(message, "CTLBN6ON")) { command.m_command = VriCommand::CtlBn6On;        return command; }
        if (startsWith(message, "CTLBN7ON")) { command.m_command = VriCommand::CtlBn7On;        return command; }

        if (startsWith(message, "CTLBN0OF")) { command.m_command = VriCommand::CtlBn0Of;        return command; }
        if (startsWith(message, "CTLBN1OF")) { command.m_command = VriCommand::CtlBn1Of;        return command; }
        if (startsWith(message, "CTLBN2OF")) { command.m_command = VriCommand::CtlBn2Of;        return command; }
        if (startsWith(message, "CTLBN3OF")) { command.m_command = VriCommand::CtlBn3Of;        return command; }
        if (startsWith(message, "CTLBN4OF")) { command.m_command = VriCommand::CtlBn4Of;        return command; }
        if (startsWith(message, "CTLBN5OF")) { command.m_command = VriCommand::CtlBn5Of;        return command; }
        if (startsWith(message, "CTLBN6OF")) { command.m_command = VriCommand::CtlBn6Of;        return command; }
        if (startsWith(message, "CTLBN7OF")) { command.m_command = VriCommand::CtlBn7Of;        return command; }
        //=======================================================================
        // Handle COM... messages
        //=======================================================================
        if (startsWith(message, "COMAUX")) { command.m_command = VriCommand::ComAux;        return command; }
        if (startsWith(message, "COMSEL1")) { command.m_command = VriCommand::ComSel1;        return command; }
        if (startsWith(message, "COMSEL2")) { command.m_command = VriCommand::ComSel2;        return command; }


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

        if (startsWith(message, "DMESEL1")) { command.m_command = VriCommand::DmeSel1;        return command; }
        if (startsWith(message, "DMESEL2")) { command.m_command = VriCommand::DmeSel2;        return command; }
        return command;
    }
    if (startsWith(message, "E")) {
        //=======================================================================
        // Handle EFI
        //=======================================================================
        if (startsWith(message, "EFIADF1")) { command.m_command = VriCommand::EfiAdf1;        return command; }
        if (startsWith(message, "EFIADF2")) { command.m_command = VriCommand::EfiAdf2;        return command; }
        if (startsWith(message, "EFIARPT")) { command.m_command = VriCommand::EfiArpt;        return command; }
        if (startsWith(message, "EFIDATA")) { command.m_command = VriCommand::EfiData;        return command; }
        if (startsWith(message, "EFIFPV")) { command.m_command = VriCommand::EfiFpv;        return command; }
        if (startsWith(message, "EFIMTR")) { command.m_command = VriCommand::EfiMtr;        return command; }
        if (startsWith(message, "EFIPOS")) { command.m_command = VriCommand::EfiPos;        return command; }
        if (startsWith(message, "EFISTA")) { command.m_command = VriCommand::EfiSta;        return command; }
        if (startsWith(message, "EFITERR")) { command.m_command = VriCommand::EfiTerr;        return command; }
        if (startsWith(message, "EFIVOR1")) { command.m_command = VriCommand::EfiVor1;        return command; }
        if (startsWith(message, "EFIVOR2")) { command.m_command = VriCommand::EfiVor2;        return command; }
        if (startsWith(message, "EFIWPT")) { command.m_command = VriCommand::EfiWpt;        return command; }
        if (startsWith(message, "EFIWX")) { command.m_command = VriCommand::EfiWx;        return command; }
        return command;
    }
    if (startsWith(message, "H")) {
        //=======================================================================
        // Handle HDG
        //=======================================================================
        if (startsWith(message, "HDGHDG+")) { command.m_command = VriCommand::HdgHdgPlus;    return command; }
        if (startsWith(message, "HDGHDG-")) { command.m_command = VriCommand::HdgHdgMin;    return command; }
        if (startsWith(message, "HDGHLD+")) { command.m_command = VriCommand::HdgHldPlus;    return command; }
        if (startsWith(message, "HDGHLD-")) { command.m_command = VriCommand::HdgHldMin;    return command; }
        if (startsWith(message, "HDGSEL+")) { command.m_command = VriCommand::HdgSelPlus;    return command; }
        if (startsWith(message, "HDGSEL-")) { command.m_command = VriCommand::HdgSelMin;        return command; }


        return command;
    }
    if (startsWith(message, "M")) {
        //=======================================================================
        // handle Min
        //=======================================================================
        if (startsWith(message, "MIN+")) { command.m_command = VriCommand::MinPlus;        return command; }
        if (startsWith(message, "MIN-")) { command.m_command = VriCommand::MinMin;        return command; }
        if (startsWith(message, "MINSEL+")) { command.m_command = VriCommand::MinSelPlus;    return command; }
        if (startsWith(message, "MINSEL-")) { command.m_command = VriCommand::MinSelMin;    return command; }
        return command;
    }
    if (startsWith(message, "N")) {

        if (startsWith(message, "NDM")) {

            //=======================================================================
            // handle NDM more prio
            //=======================================================================
            if (startsWith(message, "NDM+")) { command.m_command = VriCommand::NdmPlus;            return command; }
            if (startsWith(message, "NDM-")) { command.m_command = VriCommand::NdmMin;            return command; }
            if (startsWith(message, "NDMSEL+")) { command.m_command = VriCommand::NdmSelPlus;    return command; }
            if (startsWith(message, "NDMSEL-")) { command.m_command = VriCommand::NdmSelMin;    return command; }
            return command;
        }

        if (startsWith(message, "NDR")) {
            //=======================================================================
            // handle NDR more prio
            //=======================================================================
            if (startsWith(message, "NDR+")) { command.m_command = VriCommand::NdrPlus;            return command; }
            if (startsWith(message, "NDR-")) { command.m_command = VriCommand::NdrMin;            return command; }
            if (startsWith(message, "NDRSEL+")) { command.m_command = VriCommand::NdrSelPlus;    return command; }
            if (startsWith(message, "NDRSEL-")) { command.m_command = VriCommand::NdrSelMin;    return command; }

            return command;
        }

        //=======================================================================
        // Handle NAV... messages
        //=======================================================================
        if (startsWith(message, "NAVAUX")) { command.m_command = VriCommand::NavAux;        return command; }
        if (startsWith(message, "NAVSEL1")) { command.m_command = VriCommand::NavSel1;        return command; }
        if (startsWith(message, "NAVSEL2")) { command.m_command = VriCommand::NavSel2;        return command; }


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


        return command;
    }
    if (startsWith(message, "O")) {
        //=======================================================================
        // handle OBS
        //=======================================================================
        if (startsWith(message, "OBS+")) { command.m_command = VriCommand::OBSPlus;            return command; }
        if (startsWith(message, "OBS-")) { command.m_command = VriCommand::OBSMin;            return command; }
        if (startsWith(message, "OBSSEL+")) { command.m_command = VriCommand::OBSSelPlus;    return command; }
        if (startsWith(message, "OBSSEL-")) { command.m_command = VriCommand::OBSSelMin;    return command; }
        return command;
    }



    if (startsWith(message, "S")) {
        //=======================================================================
        // handle SPD
        //=======================================================================
        if (startsWith(message, "SPDLVL+")) { command.m_command = VriCommand::SpdLvlPlus;    return command; }
        if (startsWith(message, "SPDLVL-")) { command.m_command = VriCommand::SpdLvlMin;    return command; }

        if (startsWith(message, "SPDSPD+")) { command.m_command = VriCommand::SpdSpdMin;    return command; }
        if (startsWith(message, "SPDSPD-")) { command.m_command = VriCommand::SpdSpdPlus;    return command; }
        if (startsWith(message, "SPDSEL+")) { command.m_command = VriCommand::SpdSelPlus;    return command; }
        if (startsWith(message, "SPDSEL-")) { command.m_command = VriCommand::SpdSelMin;    return command; }


        return command;
    }
    if (startsWith(message, "T")) {
        //=======================================================================
        // Handle TRN... messages
        //=======================================================================
        if (startsWith(message, "TRNAUX")) { command.m_command = VriCommand::TrnAux;        return command; }
        if (startsWith(message, "TRNSEL")) { command.m_command = VriCommand::TrnSel;        return command; }
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
        if (startsWith(message, "VVSHLD+")) { command.m_command = VriCommand::VvsHldPlus;        return command; }
        if (startsWith(message, "VVSHLD-")) { command.m_command = VriCommand::VvsHldMin;        return command; }
        if (startsWith(message, "VVS+")) { command.m_command = VriCommand::VvsPlus;            return command; }
        if (startsWith(message, "VVS-")) { command.m_command = VriCommand::VvsMin;            return command; }
        if (startsWith(message, "VVSSEL+")) { command.m_command = VriCommand::VvsSelPlus;            return command; }
        if (startsWith(message, "VVSSEL-")) { command.m_command = VriCommand::VvsSelMin;            return command; }
    }
    return command;

}

std::string BaseDeviceHandler::ReplaceString(const std::string source_string, const std::string old_substring, const std::string new_substring)
{
    // Can't replace nothing.
    if (old_substring.empty())
        return source_string;

    // Find the first occurrence of the substring we want to replace.
    size_t substring_position = source_string.find(old_substring);

    // If not found, there is nothing to replace.
    if (substring_position == std::string::npos)
        return source_string;

    // Return the part of the source string until the first occurance of the old substring + the new replacement substring + the result of the same function on the remainder.
    return source_string.substr(0, substring_position) + new_substring + ReplaceString(source_string.substr(substring_position + old_substring.length(), source_string.length() - (substring_position + old_substring.length())), old_substring, new_substring);
}

BaseDeviceHandler::VriCommandParameters BaseDeviceHandler::parseMCP2A(string message)
{
    VriCommandParameters command;
    command.m_boosted = false;
    command.m_value = 0;
    command.m_command = VriCommand::None;

    VLTrace("parseCombo1: message %s", message.c_str());
    if (startsWith(message, "EFI")) {
        if (startsWith(message, "EFIBAR +")) { command.m_command = VriCommand::BarPlus;        return command; }
        if (startsWith(message, "EFIBAR -")) { command.m_command = VriCommand::BarMin;        return command; }
        if (startsWith(message, "EFIBAR *")) { command.m_command = VriCommand::BarSelPlus;    return command; }
        if (startsWith(message, "EFIBAR *")) { command.m_command = VriCommand::BarSelMin;    return command; }

        if (startsWith(message, "EFIhPa .")) { command.m_command = VriCommand::EfiHpa;        return command; }
        if (startsWith(message, "EFIInHg.")) { command.m_command = VriCommand::EfiInHg;        return command; }
        if (startsWith(message, "EFIILS .")) { command.m_command = VriCommand::EfiILS;        return command; }
        if (startsWith(message, "EFIFD  .")) { command.m_command = VriCommand::AplFdPlus;   return command; }
        if (startsWith(message, "EFICSTR.")) { command.m_command = VriCommand::EfiTerr;        return command; }

        if (startsWith(message, "EFINDB .")) { command.m_command = VriCommand::EfiSta;        return command; }
        if (startsWith(message, "EFIWPT .")) { command.m_command = VriCommand::EfiWpt;        return command; }
        if (startsWith(message, "EFIVORD.")) { command.m_command = VriCommand::EfiWx;        return command; }
        if (startsWith(message, "EFIARPT.")) { command.m_command = VriCommand::EfiArpt;        return command; }



        if (startsWith(message, "EFINDM +")) { command.m_command = VriCommand::NdmPlus;        return command; }
        if (startsWith(message, "EFINDM -")) { command.m_command = VriCommand::NdmMin;        return command; }
        if (startsWith(message, "EFINDM *")) { command.m_command = VriCommand::NdmSelPlus;    return command; }

        if (startsWith(message, "EFINDR +")) { command.m_command = VriCommand::NdrPlus;        return command; }
        if (startsWith(message, "EFINDR -")) { command.m_command = VriCommand::NdrMin;        return command; }
        if (startsWith(message, "EFINDR *")) { command.m_command = VriCommand::NdrSelPlus;    return command; }

        if (startsWith(message, "EFIADF1:")) { command.m_command = VriCommand::EfiAdf1;            return command; }
        if (startsWith(message, "EFIADF1.")) { command.m_command = VriCommand::EfiAdf1;            return command; }
        if (startsWith(message, "EFIVOR1:")) { command.m_command = VriCommand::EfiVor1;            return command; }
        if (startsWith(message, "EFIVOR1.")) { command.m_command = VriCommand::EfiVor1;            return command; }

        if (startsWith(message, "EFIADF2:")) { command.m_command = VriCommand::EfiAdf2;            return command; }
        if (startsWith(message, "EFIADF2.")) { command.m_command = VriCommand::EfiAdf2;            return command; }
        if (startsWith(message, "EFIVOR2:")) { command.m_command = VriCommand::EfiVor2;            return command; }
        if (startsWith(message, "EFIVOR2.")) { command.m_command = VriCommand::EfiVor2;            return command; }

        return command;
    }
    if (startsWith(message, "FCUS")) {
        if (startsWith(message, "FCUSPD +")) { command.m_command = VriCommand::SpdXXXPlus;        return command; }
        if (startsWith(message, "FCUSPD -")) { command.m_command = VriCommand::SpdXXXMin;        return command; }
        if (startsWith(message, "FCUSPD *")) { command.m_command = VriCommand::SpdSelPlus;        return command; }
        if (startsWith(message, "FCUSPD ^")) { command.m_command = VriCommand::SpdSelMin;        return command; }
        if (startsWith(message, "FCUSPD++")) { command.m_command = VriCommand::SpdSelMin;        return command; }
        if (startsWith(message, "FCUSPD--")) { command.m_command = VriCommand::SpdSelMin;        return command; }
        

        return command;
    }
    if (startsWith(message, "FCUH")) {
        if (startsWith(message, "FCUHDG -")) { command.m_command = VriCommand::HdgXXXMin;        return command; }
        if (startsWith(message, "FCUHDG +")) { command.m_command = VriCommand::HdgXXXPlus;        return command; }
        if (startsWith(message, "FCUHDG *")) { command.m_command = VriCommand::HdgSelMin;        return command; }
        if (startsWith(message, "FCUHDG ^")) { command.m_command = VriCommand::HdgSelPlus;        return command; }
        if (startsWith(message, "FCUMACH.^")) { command.m_command = VriCommand::FcuMach;        return command; }

        
        return command;
    }

    if (startsWith(message, "FCULOC .")) { command.m_command = VriCommand::HdgHdgPlus;        return command; }

    if (startsWith(message, "FCUA")) {
        if (startsWith(message, "FCUAP1 .")) { command.m_command = VriCommand::HdgHldPlus;        return command; }
        if (startsWith(message, "FCUAP2 .")) { command.m_command = VriCommand::VvsHldPlus;        return command; }
        if (startsWith(message, "FCUATHR.")) { command.m_command = VriCommand::AplAtPlus;        return command; }
        if (startsWith(message, "FCUALT -")) { command.m_command = VriCommand::AltXXXMin;        return command; }
        if (startsWith(message, "FCUALT +")) { command.m_command = VriCommand::AltXXXPlus;        return command; }
        if (startsWith(message, "FCUALT *")) { command.m_command = VriCommand::AltSelMin;        return command; }
        if (startsWith(message, "FCUALT ^")) { command.m_command = VriCommand::AltSelPlus;        return command; }
        if (startsWith(message, "FCUAPPR.")) { command.m_command = VriCommand::AplAppPlus;        return command; }

        return command;
    }
    if (startsWith(message, "FCUV")) {
        if (startsWith(message, "FCUVVS +")) { command.m_command = VriCommand::VvsPlus;            return command; }
        if (startsWith(message, "FCUVVS -")) { command.m_command = VriCommand::VvsMin;            return command; }
        if (startsWith(message, "FCUVVS *")) { command.m_command = VriCommand::VvsSelPlus;        return command; }

        return command;
    }
    if (startsWith(message, "FCUC")) {
        if (startsWith(message, "FCUCMDA")) { command.m_command = VriCommand::AplCmdAPlus;        return command; }
        if (startsWith(message, "FCUCMDB")) { command.m_command = VriCommand::AplCmdBPlus;        return command; }
        if (startsWith(message, "FCUCMDC")) { command.m_command = VriCommand::AplCmdBPlus;        return command; }
        return command;
    }

    if (startsWith(message, "FCUTO")) { command.m_command = VriCommand::AplTogaPlus;        return command; }


    if (startsWith(message, "USRB")) {

        //=======================================================================
        // handle ControlButtons
        //=======================================================================

        if (startsWith(message, "USRBTN1.")) { command.m_command = VriCommand::CtlBn0Of;        return command; }
        if (startsWith(message, "USRBTN2.")) { command.m_command = VriCommand::CtlBn1Of;        return command; }
        if (startsWith(message, "USRBTN3.")) { command.m_command = VriCommand::CtlBn2Of;        return command; }
        if (startsWith(message, "USRBTN4.")) { command.m_command = VriCommand::CtlBn3Of;        return command; }
        if (startsWith(message, "USRBTN5.")) { command.m_command = VriCommand::CtlBn4Of;        return command; }
        if (startsWith(message, "USRBTN6.")) { command.m_command = VriCommand::CtlBn5Of;        return command; }
        if (startsWith(message, "USRBTN7.")) { command.m_command = VriCommand::CtlBn6Of;        return command; }
        if (startsWith(message, "USRBTN8.")) { command.m_command = VriCommand::CtlBn7Of;        return command; }
        return command;
    }
    if (startsWith(message, "USRU")) {

        if (startsWith(message, "USRUSR1:")) { command.m_command = VriCommand::CtlBn0On;        return command; }
        if (startsWith(message, "USRUSR2:")) { command.m_command = VriCommand::CtlBn1On;        return command; }
        if (startsWith(message, "USRUSR3:")) { command.m_command = VriCommand::CtlBn2On;        return command; }
        if (startsWith(message, "USRUSR4:")) { command.m_command = VriCommand::CtlBn3On;        return command; }
        if (startsWith(message, "USRUSR6:")) { command.m_command = VriCommand::CtlBn4On;        return command; }
        if (startsWith(message, "USRUSR6:")) { command.m_command = VriCommand::CtlBn5On;        return command; }
        if (startsWith(message, "USRUSR7:")) { command.m_command = VriCommand::CtlBn6On;        return command; }
        if (startsWith(message, "USRUSR8:")) { command.m_command = VriCommand::CtlBn7On;        return command; }
        return command;
    }

    if (startsWith(message, "RAD")) {
        if (startsWith(message, "RADFRE -")) { command.m_command = VriCommand::RADFRE_MIN;            return command; }
        if (startsWith(message, "RADFRE--")) { command.m_command = VriCommand::RADFRE_MINMIN;        return command; }
        if (startsWith(message, "RADFRE +")) { command.m_command = VriCommand::RADFRE_MINMIN;        return command; }
        if (startsWith(message, "RADFRE++")) { command.m_command = VriCommand::RADFRE_PLUSPLUS;        return command; }
        if (startsWith(message, "RADCOM")) { command.m_command = VriCommand::RADCOM;        return command; }
        if (startsWith(message, "RADNAV")) { command.m_command = VriCommand::RADNAV;        return command; }
        if (startsWith(message, "RADADF")) { command.m_command = VriCommand::RADADF;        return command; }
        if (startsWith(message, "RADDME")) { command.m_command = VriCommand::RADDME;        return command; }
        if (startsWith(message, "RADTRAN")) { command.m_command = VriCommand::RADTRAN;        return command; }
        if (startsWith(message, "RADTFR")) { command.m_command = VriCommand::RADTFR;        return command; }
    }

    return command;

}

BaseDeviceHandler::VriCommandParameters BaseDeviceHandler::parseMCP2B(string message)
{
    VriCommandParameters command;
    command.m_boosted = false;
    command.m_value = 0;
    command.m_command = VriCommand::None;



    VLTrace("parseCombo2:  message %s", message.c_str());

    //priority to the main rotary buttons
    if (startsWith(message, "MCPSPD++")) { command.m_command = VriCommand::SpdXXXMin;        return command; }
    if (startsWith(message, "MCPSPD--")) { command.m_command = VriCommand::SpdXXXMin;        return command; }

    if (startsWith(message, "MCPHDG++")) { command.m_command = VriCommand::HdgXXXPlus;        return command; }
    if (startsWith(message, "MCPHDG--")) { command.m_command = VriCommand::HdgXXXMin;        return command; }
    if (startsWith(message, "MCPHDG +")) { command.m_command = VriCommand::HdgHdgPlus;        return command; }
    if (startsWith(message, "MCPHDG -")) { command.m_command = VriCommand::HdgHdgMin;        return command; }

    if (startsWith(message, "MCPALT++")) { command.m_command = VriCommand::AltXXXPlus;        return command; }
    if (startsWith(message, "MCPALT--")) { command.m_command = VriCommand::AltXXXMin;        return command; }
    if (startsWith(message, "MCPALT +")) { command.m_command = VriCommand::AltXXXPlus;        return command; }
    if (startsWith(message, "MCPALT -")) { command.m_command = VriCommand::AltXXXMin;        return command; }

    if (startsWith(message, "MCPVVS++")) { command.m_command = VriCommand::VvsPlus;        return command; }
    if (startsWith(message, "MCPVVS--")) { command.m_command = VriCommand::VvsMin;            return command; }
    if (startsWith(message, "MCPVVS +")) { command.m_command = VriCommand::VvsPlus;        return command; }
    if (startsWith(message, "MCPVVS -")) { command.m_command = VriCommand::VvsMin;            return command; }


    if (startsWith(message, "MCPA")) {

        //=======================================================================
        // Handle ALT
        //=======================================================================


        if (startsWith(message, "MCPALT *")) { command.m_command = VriCommand::AltSelMin;    return command; }
        if (startsWith(message, "MCPALT *")) { command.m_command = VriCommand::AltSelPlus;    return command; }

        if (startsWith(message, "MCPAHLD")) { command.m_command = VriCommand::AltHldPlus;    return command; }
        if (startsWith(message, "MCPAHLD")) { command.m_command = VriCommand::AltHldMin;    return command; }

        //=======================================================================
        // Handle APL
        //=======================================================================
        if (startsWith(message, "MCPAPP .")) { command.m_command = VriCommand::AplAppPlus;        return command; }
        if (startsWith(message, "MCPAPP  :")) { command.m_command = VriCommand::AplAppMin;        return command; }

        if (startsWith(message, "MCPAT  .")) { command.m_command = VriCommand::AplAtPlus;        return command; }
        if (startsWith(message, "MCPAT  :")) { command.m_command = VriCommand::AplAtMin;        return command; }

        return command;
    }

    if (startsWith(message, "MCPC")) {


        if (startsWith(message, "MCPCMDA.")) { command.m_command = VriCommand::AplCmdAPlus;    return command; }
        if (startsWith(message, "MCPCMDA.")) { command.m_command = VriCommand::AplCmdAMin;        return command; }

        if (startsWith(message, "MCPCMDB.")) { command.m_command = VriCommand::AplCmdBPlus;    return command; }
        if (startsWith(message, "MCPCMDB.")) { command.m_command = VriCommand::AplCmdBMin;        return command; }

        if (startsWith(message, "MCPCMDC.")) { command.m_command = VriCommand::AplCmdCPlus;        return command; }
        if (startsWith(message, "MCPCMDC.")) { command.m_command = VriCommand::AplCmdCMin;        return command; }

        if (startsWith(message, "MCPCWSA.")) { command.m_command = VriCommand::AplCwsAPlus;        return command; }
        if (startsWith(message, "MCPCWSA.")) { command.m_command = VriCommand::AplCwsAMin;        return command; }

        if (startsWith(message, "MCPCWSB.")) { command.m_command = VriCommand::AplCwsBPlus;        return command; }
        if (startsWith(message, "MCPCWSB.")) { command.m_command = VriCommand::AplCwsBMin;        return command; }

        return command;
    }

    if (startsWith(message, "MCPF")) {

        if (startsWith(message, "MCPFD  .")) { command.m_command = VriCommand::AplFdPlus;          return command; }
        if (startsWith(message, "MCPFD  :")) { command.m_command = VriCommand::AplFdMin;            return command; }
        return command;
    }

    if (startsWith(message, "MCPL")) {
        if (startsWith(message, "MCPLOC .")) { command.m_command = VriCommand::AplLocPlus;        return command; }
        if (startsWith(message, "MCPLOC .")) { command.m_command = VriCommand::AplLocMin;        return command; }
        if (startsWith(message, "MCPLNAV.")) { command.m_command = VriCommand::AplLnavPlus;    return command; }
        if (startsWith(message, "MCPLNAV.")) { command.m_command = VriCommand::AplLnavMin;    return command; }
        return command;
    }
    if (startsWith(message, "MCPL")) {

        if (startsWith(message, "MCPGA  .")) { command.m_command = VriCommand::AplTogaPlus;        return command; }
        if (startsWith(message, "MCPGA  .")) { command.m_command = VriCommand::AplTogaMin;        return command; }
        return command;
    }

    if (startsWith(message, "MCPT")) {

        if (startsWith(message, "MCPTO  .")) { command.m_command = VriCommand::AplTogbPlus;        return command; }
        if (startsWith(message, "MCPTO  .")) { command.m_command = VriCommand::AplTogbMin;        return command; }
    }


    if (startsWith(message, "E")) {
        if (startsWith(message, "EFIMIN")) {

            if (startsWith(message, "EFIMIN++")) { command.m_command = VriCommand::MinPlus;        return command; }
            if (startsWith(message, "EFIMIN--")) { command.m_command = VriCommand::MinMin;        return command; }
            if (startsWith(message, "EFIMIN +")) { command.m_command = VriCommand::MinSelPlus;    return command; }
            if (startsWith(message, "EFIMIN -")) { command.m_command = VriCommand::MinSelMin;    return command; }
            if (startsWith(message, "EFIMIN *")) { command.m_command = VriCommand::vMinReset;    return command; }
            return command;
        }

        if (startsWith(message, "EFIBAR")) {
            if (startsWith(message, "EFIBAR++")) { command.m_command = VriCommand::BarPlus;        return command; }
            if (startsWith(message, "EFIBAR--")) { command.m_command = VriCommand::BarMin;        return command; }
            if (startsWith(message, "EFIBAR +")) { command.m_command = VriCommand::BarSelPlus;    return command; }
            if (startsWith(message, "EFIBAR -")) { command.m_command = VriCommand::BarSelMin;    return command; }
            if (startsWith(message, "EFIBAR *")) { command.m_command = VriCommand::vBarStd;        return command; }
            return command;
        }

        if (startsWith(message, "EFINDR")) {

            //=======================================================================
            // handle NDR
            //=======================================================================
            if (startsWith(message, "EFINDR++")) { command.m_command = VriCommand::NdrPlus;            return command; }
            if (startsWith(message, "EFINDR--")) { command.m_command = VriCommand::NdrMin;            return command; }
            if (startsWith(message, "EFINDR +")) { command.m_command = VriCommand::NdrSelPlus;    return command; }
            if (startsWith(message, "EFINDR -")) { command.m_command = VriCommand::NdrSelMin;    return command; }
            return command;
        }

        if (startsWith(message, "EFINDM")) {
            //=======================================================================
            // handle NDM
            //=======================================================================
            if (startsWith(message, "EFINDM++")) { command.m_command = VriCommand::NdmPlus;        return command; }
            if (startsWith(message, "EFINDM--")) { command.m_command = VriCommand::NdmMin;        return command; }
            if (startsWith(message, "EFINDM +")) { command.m_command = VriCommand::NdmSelPlus;    return command; }
            if (startsWith(message, "EFINDM -")) { command.m_command = VriCommand::NdmSelMin;    return command; }
            //    if (startsWith(message, "EFINDM *")) { command.m_command = VriCommand::NdmSelMin;    return command; }
            return command;
        }

        if (startsWith(message, "EFIVOR")) {
            if (startsWith(message, "EFIVORL:")) { command.m_command = VriCommand::EfiAdf1;        return command; }
            if (startsWith(message, "EFIVORR:")) { command.m_command = VriCommand::EfiAdf2;        return command; }
            if (startsWith(message, "EFIVORL.")) { command.m_command = VriCommand::EfiVor1;        return command; }
            if (startsWith(message, "EFIVORR.")) { command.m_command = VriCommand::EfiVor2;        return command; }
            return command;
        }
        if (startsWith(message, "EFI")) {
            //=======================================================================
            // Handle EFI
            //=======================================================================

            if (startsWith(message, "EFIARPT")) { command.m_command = VriCommand::EfiArpt;        return command; }
            if (startsWith(message, "EFIDATA")) { command.m_command = VriCommand::EfiData;        return command; }
            if (startsWith(message, "EFIFPV")) { command.m_command = VriCommand::EfiFpv;        return command; }
            if (startsWith(message, "EFIMTR")) { command.m_command = VriCommand::EfiMtr;        return command; }
            if (startsWith(message, "EFIPOS")) { command.m_command = VriCommand::EfiPos;        return command; }
            if (startsWith(message, "EFISTA")) { command.m_command = VriCommand::EfiSta;        return command; }
            if (startsWith(message, "EFITERR")) { command.m_command = VriCommand::EfiTerr;        return command; }
            if (startsWith(message, "EFIWPT")) { command.m_command = VriCommand::EfiWpt;        return command; }
            if (startsWith(message, "EFIWX")) { command.m_command = VriCommand::EfiWx;        return command; }
            return command;
        }

        return command;
    }

    if (startsWith(message, "MCPH")) {

        //=======================================================================
        // Handle HDG
        //=======================================================================
        if (startsWith(message, "MCPHHLD.")) { command.m_command = VriCommand::HdgHldPlus;    return command; }
        if (startsWith(message, "MCPHHLD.")) { command.m_command = VriCommand::HdgHldMin;    return command; }
        if (startsWith(message, "MCPHDG *")) { command.m_command = VriCommand::HdgSelPlus;    return command; }
        if (startsWith(message, "MCPHDG *")) { command.m_command = VriCommand::HdgSelMin;    return command; }
        return command;

    }





    if (startsWith(message, "MCPV")) {
        //=======================================================================
        // Handle VVS... messages
        //=======================================================================
        if (startsWith(message, "MCPVHLD.")) { command.m_command = VriCommand::VvsHldPlus;        return command; }
        if (startsWith(message, "MCPVHLD.")) { command.m_command = VriCommand::VvsHldMin;        return command; }
        if (startsWith(message, "MCPVVS *")) { command.m_command = VriCommand::VvsSelPlus;        return command; }
        if (startsWith(message, "MCPVVS *")) { command.m_command = VriCommand::VvsSelMin;        return command; }

        if (startsWith(message, "MCPVNAV.")) { command.m_command = VriCommand::AplVnavMin;        return command; }
        if (startsWith(message, "MCPVNAV.")) { command.m_command = VriCommand::AplVnavPlus;        return command; }
        return command;

    }

    if (startsWith(message, "RAD")) {
        if (startsWith(message, "RADFRE -")) { command.m_command = VriCommand::RADFRE_MIN;            return command; }
        if (startsWith(message, "RADFRE--")) { command.m_command = VriCommand::RADFRE_MINMIN;        return command; }
        if (startsWith(message, "RADFRE +")) { command.m_command = VriCommand::RADFRE_MINMIN;        return command; }
        if (startsWith(message, "RADFRE++")) { command.m_command = VriCommand::RADFRE_PLUSPLUS;        return command; }
        if (startsWith(message, "RADCOM")) { command.m_command = VriCommand::RADCOM;        return command; }
        if (startsWith(message, "RADNAV")) { command.m_command = VriCommand::RADNAV;        return command; }
        if (startsWith(message, "RADADF")) { command.m_command = VriCommand::RADADF;        return command; }
        if (startsWith(message, "RADDME")) { command.m_command = VriCommand::RADDME;        return command; }
        if (startsWith(message, "RADTRAN")) { command.m_command = VriCommand::RADTRAN;        return command; }
        if (startsWith(message, "RADTFR")) { command.m_command = VriCommand::RADTFR;        return command; }
    }


    return command;

}


bool BaseDeviceHandler::has_any_digits(const std::string& s)
{
    return std::any_of(s.begin(), s.end(), ::isdigit);
}


