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
        void selectComboMode();

        void setAircraft(BaseAircraft *plane);

        virtual void sendToCom(const char* command);

        enum   IdentDisplayMode
        {
            Id_None,
            Id_Spd,
            Id_Hdg,
            Id_Alt,
        };

        enum   VriCommand
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
            AplDomeUp,
            AplDomeDown,
            AplFdMin,
            AplFdMinFo,
            AplFdPlus,
            AplFdPlusFo,
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

            // captain    first officer

            BarMin,            BarMinFo,
            BarPlus,        BarPlusFo,
            BarSelMin,        BarSelMinFo,
            BarSelPlus,        BarSelPlusFo,
            // virtual
            vBarStd,        vBarStdFo,
            vBarUpperPlus,  vBarUpperPlusFo,
            vBarLowerPlus,  vBarLowerPlusFo,
            vBarUpperMin,   vBarUpperMinFo,
            vBarLowerMin,   vBarLowerMinFo,

            ComAux,
            Coms,
            ComS,
            Com1StbyCourseDown, Com1StbyCourseUp,
            Com1StbyFineDown, Com1StbyFineUp,

            Com2StbyCourseDown, Com2StbyCourseUp,
            Com2StbyFineDown, Com2StbyFineUp,

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

            // captain    first officer

            EfiAdf1,        EfiAdf1Fo,
            EfiAdf2,        EfiAdf2Fo,
            EfiArpt,        EfiArptFo,
            EfiData,        EfiDataFo,
            EfiFpv,            EfiFpvFo,
            EfiMtr,            EfiMtrFo,
            EfiPos,            EfiPosFo,
            EfiSta,            EfiStaFo,
            EfiTerr,        EfiTerrFo,
            EfiVor1,        EfiVor1Fo,
            EfiVor2,        EfiVor2Fo,
            EfiWpt,            EfiWptFo,
            EfiWx,            EfiWxFo,



            HdgHdgMin,
            HdgHdgPlus,
            HdgHldMin,
            HdgHldPlus,
            HdgSelMin,
            HdgSelPlus,
            HdgXXXMin,
            HdgXXXPlus,

            // captain    first officer

            MinMin,        MinMinFo,
            MinPlus,       MinPlusFo,
            MinSelMin,     MinSelMinFo,

            MinSelPlus,    MinSelPlusFo,
            vMinReset,     vMinResetFo,
            vMinLowerPlus, vMinLowerPlusFo,
            vMinLowerMin,  vMinLowerMinFo,
            vMinUpperPlus, vMinUpperPlusFo,
            vMinUpperMin,  vMinUpperMinFo,

            

            NavAux,

                Nav1StbyCourseDown, Nav1StbyCourseUp,
                Nav1StbyFineDown, Nav1StbyFineUp,

                Nav2StbyCourseDown, Nav2StbyCourseUp,
                Nav2StbyFineDown, Nav2StbyFineUp,
            Navs,
            NavS,
            NavSel1,
            NavSel2,
            Navx,
            NavX,

            // captain    first officer
            NdmMin,            NdmMinFo,
            NdmPlus,        NdmPlusFo,// CTR
            NdmSelMin,        NdmSelMinFo,
            NdmSelPlus,        NdmSelPlusFo,
            NdrMin,            NdrMinFo,
            NdrPlus,        NdrPlusFo,
            NdrSelMin,        NdrSelMinFo,
            NdrSelPlus,        NdrSelPlusFo,

            

            OBSMin, OBSMinFo,
            OBSPlus, OBSPlusFo,
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

                RADCOM,
                RADNAV,
                RADADF,
                RADDME,
                RADTRAN,
                RADTFR,
                RADFRE_MIN,
                RADFRE_MINMIN,
                RADFRE_PLUS,
                RADFRE_PLUSPLUS,

                //MCP2A
                EfiHpa,
                EfiInHg,
                EfiILS,
                EfiCstr,
                FcuMach,
                

            last


            };

        enum class VriDataRef
        {
            DR_None,
            DR_TailNum,
            DR_Altitude,
            DR_CoursePilot, DR_CoursePilotFo,
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
            
            DR_FlightDirector,        DR_FlightDirectorFo,
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

            // captain    first officer
            DR_MapMode,        DR_MapModeFo,
            DR_MapRange,    DR_MapRangeFo,
            DR_Vor1Pos,        DR_Vor1PosFo,
            DR_Vor2Pos,        DR_Vor2PosFo,

            DR_Course,
            DR_CurrentAltitude,
            DR_SetAltitude,
            DR_CurrentAirspeed,
            DR_SetAirspeed,
            DR_CurrentHeading,
            DR_Priority,

            DR_BarStd,
            DR_BarStdFo,
            DR_Minimums,
            DR_MinimumsFo,

            DR_LandingLights,
            DR_TaxiLights,
            DR_NavigationLights,
            DR_BeaconLights,
            DR_StrobeLights,
            DR_ApuSwitch,


            last_dr

        };

        struct VriCommandParameters
        {
            VriCommand m_command;
            bool m_boosted;
            int m_value;
            float m_float;

        };

        enum   VriInsightEquipment
        {
            unknown,
            cmdfmer,
            cmdmcp2a,
            cmdmcp2b,
        
        };


        VriInsightEquipment vriInsightEquipment;


    protected:
        VriCommandParameters parse(string message);

        VriCommandParameters parseFMER( string message);
        VriCommandParameters parseMCP2A(string message);
        VriCommandParameters parseMCP2B(string message);
        VriCommandParameters handleCombo2Radios(string message);


        VRiCommPort *m_commPort;

    private:
        BaseAircraft *m_aircraft;

        char m_dmeDistance[16];
        char m_dmeSpeed[16];
        char m_dmeCourse[16];
        char m_dmeIdent[16];



        std::string ReplaceString(const std::string source_string, const std::string old_substring, const std::string new_substring);

        bool has_any_digits(const std::string& s);
    };

#endif
