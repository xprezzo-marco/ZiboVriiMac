#ifndef BASEDEVICEHANDLERH
#define BASEDEVICEHANDLERH
#include <string>
using namespace std;

class BaseAircraft;
class VRiCommPort;

class BaseDeviceHandler
{
public:
	BaseDeviceHandler(VRiCommPort *commPort);
	~BaseDeviceHandler();

	void parseCommand(string message);

	void setAircraft(BaseAircraft *plane);

	

	virtual void sendToCom(const char* command);

	

	enum VriCommand 
	{ 
        None,
            Adf,
            AdfAux,
            AdfSel1,
            AdfSel2,
            AltHldMin,
            AltHldPlus,
            AltSelMin,
            AltSelPlus,
            AltXXXMin,
            AltXXXPlus,
            AplAppMin,
            AplAppPlus,
            AplAtMin,
            AplAtPlus,
            AplCmdAMin,
            AplCmdAPlus,
            AplCmdBMin,
            AplCmdBPlus,
            AplCmdCMin,
            AplCmdCPlus,
            AplCwsAMin,
            AplCwsAPlus,
            AplCwsBMin,
            AplCwsBPlus,
            AplDomeOn,
            AplDomeOff,
            AplFdMin,
            AplFdPlus,
            AplLnavMin,
            AplLnavPlus,
            AplLocMin,
            AplLocPlus,
            AplMastMin,
            AplMastPlus,
            AplTogaMin,
            AplTogaPlus,
            AplTogbMin,
            AplTogbPlus,
            AplVnavMin,
            AplVnavPlus,
            BankAngleMin,
            BankAnglePlus,
            BarMin,
            BarPlus,
            BarSelMin,
            BarSelPlus,
            // virtual
            vBarStd,     vBarUpperPlus,    vBarLowerPlus,vBarUpperMin,vBarLowerMin,
            ComAux,
            Coms,
            ComS,
            ComSel1,
            ComSel2,
            Comx,
            ComX,
            Crs,
            CrsxxxMin,
            CrsxxxPlus,
            CtlBn0Of,
            CtlBn0On,
            CtlBn1Of,
            CtlBn1On,
            CtlBn2Of,
            CtlBn2On,
            CtlBn3Of,
            CtlBn3On,
            CtlBn4Of,
            CtlBn4On,
            CtlBn5Of,
            CtlBn5On,
            CtlBn6Of,
            CtlBn6On,
            CtlBn7Of,
            CtlBn7On,
            DmeAux,
            DmeSel1,
            DmeSel2,

        
            EfiAdf1,
            EfiAdf2,
            EfiArpt,
            EfiData,
            EfiFpv,
            EfiMtr,
            EfiPos,
            EfiSta,
            EfiTerr,
            EfiVor1,
            EfiVor2,
            EfiWpt,
            EfiWx,
            HdgHdgMin,
            HdgHdgPlus,
            HdgHldMin,
            HdgHldPlus,
            HdgSelMin,
            HdgSelPlus,
            HdgXXXMin,
            HdgXXXPlus,
            MinMin,
            MinPlus,
            MinSelMin,
            MinSelPlus,
            vMinReset,    vMinLowerPlus,    vMinLowerMin,vMinUpperPlus,    vMinUpperMin,// virtual command

            NavAux,
            Navs,
            NavS,
            NavSel1,
            NavSel2,
            Navx,
            NavX,
            NdmMin,
            NdmPlus, // CTR
            NdmSelMin,
            NdmSelPlus,
            NdrMin,
            NdrPlus, // TFC
            NdrSelMin,
            NdrSelPlus,
            OBSMin,
            OBSPlus,// CRS
            OBSSelMin,
            OBSSelPlus,
            SpdLvlMin,
            SpdLvlPlus,
            SpdN1Min,
            SpdN1Plus,
            SpdSelMin,
            SpdSelPlus,
            SpdSpdMin,
            SpdSpdPlus,
            SpdXXXMin,
            SpdXXXPlus,
            TrnAux,
            Trns,
            TrnSel,
            Trnx,
            VvsHldMin,
            VvsHldPlus,
            VvsMin,
            VvsPlus,
            VvsSelMin,
            VvsSelPlus,


           
		};

	enum  VriDataRef
	{
        DR_None,
                DR_TailNum,
                DR_Altitude,
                DR_CoursePilot,
                DR_Speed,
                DR_Heading,
                DR_Vvs,
                DR_BankAngle,

                DR_Com1,
                DR_Com1Standby,
                DR_Com2,
                DR_Com2Standby,
                DR_Nav1,
                DR_Nav1Standby,
                DR_Nav2,
                DR_Nav2Standby,
                DR_Transponder,
                DR_Dme1Dist,
                DR_Dme1Ident,
                DR_Dme1Speed,
                DR_Dme1Course,
                DR_Dme2Dist,
                DR_Dme2Ident,
                DR_Dme2Speed,
                DR_Dme2Course,
                DR_TransponderCode,
                DR_TransponderMode,
                DR_AutoThrottle,
                DR_FlightDirector,
                DR_ApMaster,
                DR_SpdN1,
                DR_SpdSpd,
                DR_SpdLvl,
                DR_HdgHdg,
                DR_HdgHld,
                DR_AltHld,
                DR_VvsHld,
                DR_AplVnav,
                DR_AplLnav,
                DR_AplCmdA,
                DR_AplCmdB,
                DR_AplCmdC,
                DR_CwsA,
                DR_CwsB,
                DR_AplApp,
                DR_AplLoc,

                DR_Adf1,
                DR_Adf1Standby,
                DR_Adf2,
                DR_Adf2Standby,

                DR_PanelBrightness,
                DR_GenericLightsSwitch,

                DR_MapMode,
                DR_MapRange,
                DR_Vor1Pos,
                DR_Vor2Pos,
                DR_Course
	

	};

	struct VriCommandParameters
	{
		VriCommand m_command;
		bool m_boosted;
		int m_value;
		float m_float;

	};

protected:
	VriCommandParameters parse( string message);

	VRiCommPort *m_commPort;

private:
	BaseAircraft *m_aircraft;

	char m_dmeDistance[16];
	char m_dmeSpeed[16];
	char m_dmeCourse[16];
	char m_dmeIdent[16];
};

#endif
