#include "BaseAircraft.h"
#include "McpDisplay.h"
#include "BaseDeviceHandler.h"


#include "logger.h"
#include <fstream>

#include <mutex>
#include <list>
#include "VRiCommPort.h"
#include <string.h>


static std::mutex commandMutex;




BaseAircraft::BaseAircraft(BaseDeviceHandler* baseDeviceHandler)
    : m_xpCommands(nullptr)
    , m_radioMode(RadioDisplay::RadioMode::Com1Standby)
{
    m_baseDeviceHandler = baseDeviceHandler;


}

BaseAircraft::~BaseAircraft()
{
}

bool BaseAircraft::initData() {


    bool ret = initDataRef();
    ret &= initCommand();

    return ret;
}
bool BaseAircraft::initDataRef() {


    VLLog("initDataRef LOADING...");
    drMap.clear();

    bool ret = true;

    //ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_TailNum,            "sim/aircraft/view/acf_tailnum");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Altitude, "sim/cockpit2/autopilot/altitude_dial_ft");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_CoursePilot, "laminar/B738/autopilot/course_pilot");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Speed, "sim/cockpit2/autopilot/airspeed_dial_kts_mach");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Heading, "sim/cockpit2/autopilot/heading_dial_deg_mag_pilot");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Vvs, "sim/cockpit2/autopilot/vvi_dial_fpm");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Com1, "sim/cockpit/radios/com1_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Com1Standby, "sim/cockpit/radios/com1_stdby_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Com2, "sim/cockpit/radios/com2_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Com2Standby, "sim/cockpit/radios/com2_stdby_freq_hz");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Nav1, "sim/cockpit/radios/nav1_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Nav1Standby, "sim/cockpit/radios/nav1_stdby_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Nav2, "sim/cockpit/radios/nav2_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Nav2Standby, "sim/cockpit/radios/nav2_stdby_freq_hz");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Dist, "sim/cockpit2/radios/indicators/nav1_dme_distance_nm");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Ident, "sim/cockpit2/radios/indicators/nav1_nav_id");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Speed, "sim/cockpit2/radios/indicators/nav1_dme_speed_kts");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Course, "sim/cockpit/radios/nav1_obs_degm");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Dist, "sim/cockpit2/radios/indicators/nav2_dme_distance_nm");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Ident, "sim/cockpit2/radios/indicators/nav2_nav_id");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Speed, "sim/cockpit2/radios/indicators/nav2_dme_speed_kts");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Course, "sim/cockpit/radios/nav2_obs_degm");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_TransponderCode, "sim/cockpit2/radios/actuators/transponder_code");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_TransponderMode, "sim/cockpit2/radios/actuators/transponder_mode");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AutoThrottle, "laminar/B738/autopilot/autothrottle_arm_pos");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_FlightDirector, "laminar/B738/autopilot/flight_director_pos");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_ApMaster, "laminar/B738/autopilot/disconnect_pos");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_SpdN1, "laminar/B738/autopilot/n1_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_SpdSpd, "laminar/B738/autopilot/speed_status1");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_SpdLvl, "laminar/B738/autopilot/lvl_chg_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_HdgHdg, "laminar/B738/autopilot/hdg_sel_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_HdgHld, "sim/cockpit2/autopilot/heading_hold_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_BankAngle, "laminar/B738/rotary/autopilot/bank_angle");


    
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AltHld, "laminar/B738/autopilot/alt_hld_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_VvsHld, "laminar/B738/autopilot/disconnect_pos");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AplVnav, "laminar/B738/autopilot/vs_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AplLnav, "laminar/B738/autopilot/lnav_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AplCmdA, "laminar/B738/autopilot/cmd_a_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AplCmdB, "laminar/B738/autopilot/cmd_b_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AplCmdC, nullptr);
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_CwsA, "laminar/B738/autopilot/cws_a_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_CwsB, "laminar/B738/autopilot/cws_b_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AplApp, "laminar/B738/autopilot/app_status");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_AplLoc, "laminar/B738/autopilot/vorloc_status");




    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Adf1, "sim/cockpit/radios/adf1_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Adf1Standby, "sim/cockpit/radios/adf1_stdby_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Adf2, "sim/cockpit/radios/adf2_freq_hz");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Adf2Standby, "sim/cockpit/radios/adf2_stdby_freq_hz");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_PanelBrightness, "laminar/B738/electric/panel_brightness");
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_GenericLightsSwitch, "sim/cockpit2/switches/generic_lights_switch");

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_MapMode, "laminar/B738/EFIS_control/capt/map_mode_pos"); //Map mode. 0 = approach, 1 = vor, 2 = map, 3 = nav, 4 = plan"
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_MapRange, "laminar/B738/EFIS/capt/map_range"); //Map range, 0-7."

    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Vor1Pos, "laminar/B738/EFIS_control/capt/vor1_off_pos"); //1=VOR On 0=VOR off -1=ADf."
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Vor2Pos, "laminar/B738/EFIS_control/capt/vor2_off_pos"); //1=VOR On 0=VOR off -1=ADf."
    ret &= storeDataRef(BaseDeviceHandler::VriDataRef::DR_Course, "sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot"); //course


    if (!ret) {
        VLError("InitDataRef failed");
        return false;
    }

    VLLog("InitDataRef loaded succesfully %d records", drMap.size());
    return true;

}

bool BaseAircraft::initCommand() {

    cmdMap.clear();

    VLLog("initCommand LOADING...");
    bool ret = true;

    ret &= storeCommand(BaseDeviceHandler::VriCommand::Adf, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AdfAux, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AdfSel1, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AdfSel2, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AltHldMin, "laminar/B738/autopilot/alt_hld_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AltHldPlus, "laminar/B738/autopilot/alt_hld_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AltSelMin, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AltSelPlus, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AltXXXMin, "laminar/B738/autopilot/altitude_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AltXXXPlus, "laminar/B738/autopilot/altitude_up");


    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplAppMin, "laminar/B738/autopilot/app_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplAppPlus, "laminar/B738/autopilot/app_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplAtMin, "laminar/B738/autopilot/autothrottle_arm_toggle");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplAtPlus, "laminar/B738/autopilot/autothrottle_arm_toggle");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCmdAMin, "laminar/B738/autopilot/cmd_a_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCmdAPlus, "laminar/B738/autopilot/cmd_a_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCmdBMin, "laminar/B738/autopilot/cmd_b_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCmdBPlus, "laminar/B738/autopilot/cmd_b_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCmdCMin, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCmdCPlus, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplDomeOn, "laminar/B738/toggle_switch/cockpit_dome_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplDomeOff, "laminar/B738/toggle_switch/cockpit_dome_dn");


    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCwsAMin, "laminar/B738/autopilot/cws_a_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCwsAPlus, "laminar/B738/autopilot/cws_a_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCwsBMin, "laminar/B738/autopilot/cws_b_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplCwsBPlus, "laminar/B738/autopilot/cws_b_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplFdMin, "laminar/B738/autopilot/flight_director_toggle");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplFdPlus, "laminar/B738/autopilot/flight_director_toggle");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplLnavMin, "laminar/B738/autopilot/lnav_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplLnavPlus, "laminar/B738/autopilot/lnav_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplLocMin, "laminar/B738/autopilot/vorloc_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplLocPlus, "laminar/B738/autopilot/vorloc_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplMastMin, "laminar/B738/autopilot/disconnect_toggle");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplMastPlus, "laminar/B738/autopilot/disconnect_toggle");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplTogaMin, "laminar/B738/autopilot/right_toga_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplTogaPlus, "laminar/B738/autopilot/right_toga_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplTogbMin, "laminar/B738/autopilot/left_toga_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplTogbPlus, "laminar/B738/autopilot/left_toga_press");


    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplVnavMin, "laminar/B738/autopilot/vnav_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::AplVnavPlus, "laminar/B738/autopilot/vnav_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::BankAngleMin, "laminar/B738/autopilot/bank_angle_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::BankAnglePlus, "laminar/B738/autopilot/bank_angle_up");



    ret &= storeCommand(BaseDeviceHandler::VriCommand::vBarLowerMin, "laminar/B738/EFIS_control/capt/baro_in_hpa_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::vBarLowerPlus, "laminar/B738/EFIS_control/capt/baro_in_hpa_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::vBarUpperMin, "laminar/B738/pilot/barometer_down");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::vBarUpperPlus, "laminar/B738/pilot/barometer_up");


    ret &= storeCommand(BaseDeviceHandler::VriCommand::vBarStd, "sim/instruments/barometer_2992");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::BarSelMin, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::BarSelPlus, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::ComAux, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::Coms, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::ComS, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::ComSel1, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::ComSel2, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::Comx, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::ComX, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::Crs, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CrsxxxMin, "sim/radios/obs_HSI_down");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CrsxxxPlus, "sim/radios/obs_HSI_up");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn0Of, "sim/lights/landing_lights_off");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn0On, "sim/lights/landing_lights_on");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn1Of, "laminar/B738/toggle_switch/taxi_light_brightness_pos_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn1On, "laminar/B738/toggle_switch/taxi_light_brightness_pos_dn");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn2Of, "laminar/B738/switch/logo_light_off");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn2On, "laminar/B738/switch/logo_light_on");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn3Of, "laminar/B738/switch/wing_light_off");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn3On, "laminar/B738/switch/wing_light_on");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn4Of, "laminar/B738/switch/wheel_light_off");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn4On, "laminar/B738/switch/wheel_light_on");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn5Of, "laminar/B738/spring_toggle_switch/APU_start_pos_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn5On, "laminar/B738/spring_toggle_switch/APU_start_pos_up");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn6Of, "sim/flight_controls/landing_gear_down");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn6On, "sim/flight_controls/landing_gear_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn7Of, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::CtlBn7On, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::DmeAux, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::DmeSel1, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::DmeSel2, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiAdf1, "laminar/B738/EFIS_control/capt/vor1_off_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiAdf2, "laminar/B738/EFIS_control/capt/vor2_off_dn");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiArpt, "laminar/B738/EFIS_control/capt/push_button/arpt_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiData, "laminar/B738/EFIS_control/capt/push_button/data_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiFpv, "laminar/B738/EFIS_control/capt/push_button/fpv_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiMtr, "laminar/B738/EFIS_control/capt/push_button/mtrs_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiPos, "laminar/B738/EFIS_control/capt/push_button/pos_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiSta, "laminar/B738/EFIS_control/capt/push_button/sta_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiTerr, "laminar/B738/EFIS_control/capt/push_button/terr_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiVor1, "laminar/B738/EFIS_control/capt/vor1_off_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiVor2, "laminar/B738/EFIS_control/capt/vor2_off_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiWpt, "laminar/B738/EFIS_control/capt/push_button/wpt_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::EfiWx, "laminar/B738/EFIS_control/capt/push_button/wxr_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgHdgMin, "laminar/B738/autopilot/hdg_sel_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgHdgPlus, "laminar/B738/autopilot/hdg_sel_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgHldMin, "sim/autopilot/heading_sync");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgHldPlus, "sim/autopilot/heading_sync");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgSelMin, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgSelPlus, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgXXXMin, "sim/autopilot/heading_down");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::HdgXXXPlus, "sim/autopilot/heading_up");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::vMinLowerPlus, "laminar/B738/EFIS_control/cpt/minimums_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::vMinLowerMin, "laminar/B738/EFIS_control/cpt/minimums_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::vMinUpperMin, "laminar/B738/pfd/dh_pilot_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::vMinUpperPlus, "laminar/B738/pfd/dh_pilot_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::vMinReset, "laminar/B738/EFIS_control/capt/push_button/rst_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::MinSelPlus, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::NavAux, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::Navs, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NavS, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NavSel1, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NavSel2, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::Navx, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NavX, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdmMin, "laminar/B738/EFIS_control/capt/map_mode_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdmPlus, "laminar/B738/EFIS_control/capt/map_mode_up"); // CTR
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdmSelMin, "laminar/B738/EFIS_control/capt/push_button/ctr_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdmSelPlus, "laminar/B738/EFIS_control/capt/push_button/ctr_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdrMin, "laminar/B738/EFIS_control/capt/map_range_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdrPlus, "laminar/B738/EFIS_control/capt/map_range_up"); // TFC
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdrSelMin, "laminar/B738/EFIS_control/capt/push_button/tfc_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::NdrSelPlus, "laminar/B738/EFIS_control/capt/push_button/tfc_press");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::OBSMin, "sim/radios/obs_HSI_down");;
    ret &= storeCommand(BaseDeviceHandler::VriCommand::OBSPlus, "sim/radios/obs_HSI_up");// CRS
    ret &= storeCommand(BaseDeviceHandler::VriCommand::OBSSelMin, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::OBSSelPlus, nullptr);

    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdLvlMin, "laminar/B738/autopilot/lvl_chg_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdLvlPlus, "laminar/B738/autopilot/lvl_chg_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdN1Min, "laminar/B738/autopilot/n1_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdN1Plus, "laminar/B738/autopilot/n1_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdSelMin, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdSelPlus, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdSpdMin, "laminar/B738/autopilot/speed_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdSpdPlus, "laminar/B738/autopilot/speed_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdXXXMin, "sim/autopilot/airspeed_down");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::SpdXXXPlus, "sim/autopilot/airspeed_up");

    ret &= storeCommand(BaseDeviceHandler::VriCommand::TrnAux, "sim/transponder/transponder_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::Trns, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::TrnSel, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::Trnx, "sim/transponder/transponder_dn");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::VvsHldMin, "laminar/B738/autopilot/vs_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::VvsHldPlus, "laminar/B738/autopilot/vs_press");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::VvsMin, "sim/autopilot/vertical_speed_down");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::VvsPlus, "sim/autopilot/vertical_speed_up");
    ret &= storeCommand(BaseDeviceHandler::VriCommand::VvsSelMin, nullptr);
    ret &= storeCommand(BaseDeviceHandler::VriCommand::VvsSelPlus, nullptr);



    if (!ret) {
        VLError("InitCommand failed");
        return false;
    }

    VLLog("InitCommand loaded succesfully %d records", cmdMap.size());

    return ret;


}

bool BaseAircraft::storeCommand(BaseDeviceHandler::VriCommand vriCommand, const char* apiCommand) {

    if (apiCommand == nullptr) {
        return true;
    }

    XPLMCommandRef command = findCommandRef(apiCommand);
    if (command == nullptr) {
        return false;
    }

    if (cmdMap.count(vriCommand) > 0) {
        VLError("storeCommand: vriCommand [%d] already exists [%s]", vriCommand, apiCommand);

        return false;
    }

    VLLog("storeCommand: vriCommand [%d] apiCommand [%s]", vriCommand, apiCommand);

    cmdMap[vriCommand] = command;

    return true;
}

bool BaseAircraft::storeDataRef(BaseDeviceHandler::VriDataRef vriDataRef, const char* apiReference) {

    if (apiReference == nullptr){
        return true;
    }
    XPLMDataRef dataRef = findDataRef(apiReference);
    if (dataRef == nullptr) {
        return false;
    }

    if (drMap.count(vriDataRef) > 0) {
        VLError("storeDataRef: vriDataRef [%d] already exists [%s]", vriDataRef, apiReference);
        return false;
    }
    VLLog("storeDataRef:vriDataRef [%d] apiReference [%s]", vriDataRef, apiReference);

    drMap[vriDataRef] = dataRef;

    return true;
}

XPLMDataRef BaseAircraft::findDataRef(const char* reference)
{

    XPLMDataRef ref = XPLMFindDataRef(reference);
    if (ref == nullptr)
    {
        VLError("Could not find data reference: <%s>\n", reference);
    }
    return ref;
}

XPLMCommandRef BaseAircraft::findCommandRef(const char* command)
{



    XPLMCommandRef ref = XPLMFindCommand(command);
    if (ref == nullptr)
    {
        VLError("Could not find command reference: <%s>", command);
    }
    return ref;
}



void BaseAircraft::updateMcpDisplays()
{

    m_speed->sync();
    m_heading->sync();
    m_altitude->sync();
}


void BaseAircraft::updateIdent(const char* ident) {

    m_baseDeviceHandler->sendToCom(ident);

}





void BaseAircraft::initMcpDisplays() {


    VLLog("initMcpDisplays LOADING...");


    bool ret = true;
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Speed, drSpeed);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Heading, drHeading);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Altitude, drAltitude);

    if (!ret) {
        VLError("initMcpDisplays failed...");
    }
    m_speed = new McpDisplay(m_baseDeviceHandler, drSpeed, McpDisplay::McpMode::Speed);
    m_heading = new McpDisplay(m_baseDeviceHandler, drHeading, McpDisplay::McpMode::Heading);
    m_altitude = new McpDisplay(m_baseDeviceHandler, drAltitude, McpDisplay::McpMode::Altitude);

    updateMcpDisplays();

}

void BaseAircraft::initRadios() {


    bool ret = true;

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Com1, drCom1);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Com1Standby, drCom1Standby);

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Com2, drCom2);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Com2Standby, drCom2Standby);

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Nav1, drNav1);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Nav1Standby, drNav1Standby);

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Nav2, drNav2);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Nav2Standby, drNav2Standby);

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Adf1, drAdf1);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Adf1Standby, drAdf1Standby);

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Adf2, drAdf2);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Adf2Standby, drAdf2Standby);

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_TransponderCode, drTransponder);


    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Dist, drDme1Dist);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Speed, drfDme1Ident);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Course, drDme1Speed);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme1Ident, drDme1Course);

    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Dist, drDme2Dist);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Speed, drfDme2Ident);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Course, drDme2Speed);
    ret &= getXPLMDataRef(BaseDeviceHandler::VriDataRef::DR_Dme2Ident, drDme2Course);


    if (!ret) { VLError("initRadios failed..."); }

    m_com1Act = new RadioDisplay(m_baseDeviceHandler, "COM1_ACT", drCom1, RadioDisplay::RadioMode::Com1);
    m_com1Stby = new RadioDisplay(m_baseDeviceHandler, "COM1_STBY", drCom1Standby, RadioDisplay::RadioMode::Com1Standby);

    m_com2Act = new RadioDisplay(m_baseDeviceHandler, "COM2_ACT", drCom2, RadioDisplay::RadioMode::Com2);
    m_com2Stby = new RadioDisplay(m_baseDeviceHandler, "COM2_STBY", drCom2Standby, RadioDisplay::RadioMode::Com2Standby);

    m_nav1Act = new RadioDisplay(m_baseDeviceHandler, "NAV1_ACT", drNav1, RadioDisplay::RadioMode::Nav1);
    m_nav1Stby = new RadioDisplay(m_baseDeviceHandler, "NAV1_STBY", drNav1Standby, RadioDisplay::RadioMode::Nav1Standby);

    m_nav2Act = new RadioDisplay(m_baseDeviceHandler, "NAV2_ACT", drNav2, RadioDisplay::RadioMode::Nav2);
    m_nav2Stby = new RadioDisplay(m_baseDeviceHandler, "NAV2_STBY", drNav2Standby, RadioDisplay::RadioMode::Nav2Standby);

    m_adf1Act = new RadioDisplay(m_baseDeviceHandler, "ADF1_ACT", drAdf1, RadioDisplay::RadioMode::Adf1);
    m_adf1Stby = new RadioDisplay(m_baseDeviceHandler, "ADF1_STBY", drAdf1Standby, RadioDisplay::RadioMode::Adf1StandBy);

    m_adf2Act = new RadioDisplay(m_baseDeviceHandler, "ADF2_ACT", drAdf2, RadioDisplay::RadioMode::Adf2);
    m_adf2Stby = new RadioDisplay(m_baseDeviceHandler, "ADF2_STBY", drAdf2Standby, RadioDisplay::RadioMode::Adf2StandBy);

    m_transponder = new RadioDisplay(m_baseDeviceHandler, "TRANSPONDER", drTransponder, RadioDisplay::RadioMode::Transponder);


    m_dme1 = new DmeDisplay(m_baseDeviceHandler, 1, drDme1Dist, drDme1Speed, drDme1Course, drfDme1Ident);
    m_dme2 = new DmeDisplay(m_baseDeviceHandler, 2, drDme2Dist, drDme2Speed, drDme2Course, drfDme2Ident);

    initializeRadios();


}
void BaseAircraft::initializeRadios()
{

    VLLog("initializeRadios getting settings...");

    m_com1Act->sync();
    m_com1Stby->sync();

    m_com2Act->sync();
    m_com2Stby->sync();

    m_nav1Act->sync();
    m_nav1Stby->sync();

    m_nav2Act->sync();
    m_nav2Stby->sync();

    m_adf1Act->sync(true); // only sync, there is no display
    m_adf1Stby->sync();

    m_adf2Act->sync(true);  // only sync, there is no display
    m_adf2Stby->sync();

    m_transponder->sync();


    m_dme1->sync();
    m_dme2->sync();


}
void BaseAircraft::updateRadios()
{
    switch (m_radioMode) {
    case RadioDisplay::RadioMode::Com1Standby:
    case RadioDisplay::RadioMode::Com1:
        m_com1Act->sync();
        m_com1Stby->sync();
        break;

    case RadioDisplay::RadioMode::Com2Standby:
    case RadioDisplay::RadioMode::Com2:
        m_com2Act->sync();
        m_com2Stby->sync();
        break;

    case RadioDisplay::RadioMode::Nav1Standby:

    case RadioDisplay::RadioMode::Nav1:
        m_nav1Act->sync();
        m_nav1Stby->sync();
        break;

    case RadioDisplay::RadioMode::Nav2Standby:
    case RadioDisplay::RadioMode::Nav2:
        m_nav2Act->sync();
        m_nav2Stby->sync();
        break;

    case RadioDisplay::RadioMode::Adf1StandBy:
    case RadioDisplay::RadioMode::Adf1:

        m_adf1Stby->sync();
        m_adf1Act->sync(true); // only sync, there is no display
        break;
    case RadioDisplay::RadioMode::Adf2StandBy:
    case RadioDisplay::RadioMode::Adf2:

        m_adf2Stby->sync();
        m_adf2Act->sync(true);  // only sync, there is no display
        break;

    case RadioDisplay::RadioMode::Transponder:
        m_transponder->sync();
        break;
    case RadioDisplay::RadioMode::Dme1:
        m_dme1->sync();
        break;
    case RadioDisplay::RadioMode::Dme2:
        m_dme2->sync();
        break;


    default:
        break;
    }



}

XPLMDataRef BaseAircraft::getDataRef(BaseDeviceHandler::VriDataRef vriDataRef) {
    XPLMDataRef ref;
    getXPLMDataRef(vriDataRef, ref);
    return ref;
}






bool BaseAircraft::handleCommand(BaseDeviceHandler::VriCommandParameters command)
{
    int adf1Value = -1;
    int adf2Value = -1;

    VLTrace("handleCommand: command %d ", command.m_command);
    switch (command.m_command)
    {
    case BaseDeviceHandler::VriCommand::AdfSel1:    selectRadio("ADF1 Selected", m_adf1Stby); return true;
    case BaseDeviceHandler::VriCommand::AdfSel2:    selectRadio("ADF2 Selected", m_adf2Stby); return true;
    case BaseDeviceHandler::VriCommand::Adf:

        if (command.m_value < 10) {
            //ignore since we do not support .1 Khz in xplane
            return true;
        }

        switch (m_radioMode) {
        case RadioDisplay::RadioMode::Adf1StandBy:


            adf1Value = command.m_value;

            VLLog("handleCommand: Adf1 value %d", adf1Value);

            if (m_adf1LastValue == adf1Value) {

                transferRadio("ADF1 exchange", m_adf1Act, m_adf1Stby);

                return true;
            }

            m_adf1Stby->updateVri2Xp(adf1Value);
            m_adf1LastValue = adf1Value;



            return true;

        case RadioDisplay::RadioMode::Adf2StandBy:

            adf2Value = command.m_value;

            VLLog("handleCommand: Adf2 value %d ", adf2Value);

            if (m_adf1LastValue == adf2Value) {

                transferRadio("ADF2 exchange", m_adf2Act, m_adf2Stby);
                return true;
            }

            m_adf2Stby->updateVri2Xp(adf2Value);
            m_adf1LastValue = adf2Value;


            return true;

        default:
            break;
        }
        return true;



    case BaseDeviceHandler::VriCommand::ComSel1:    selectRadio("COM 1 Selected", m_com1Stby); return true;
    case BaseDeviceHandler::VriCommand::Coms:        m_com1Stby->updateVri2Xp(command.m_value);     return true;
    case BaseDeviceHandler::VriCommand::Comx:        transferRadio("COM1 exchange", m_com1Act, m_com1Stby);    return true;

    case BaseDeviceHandler::VriCommand::ComSel2:    selectRadio("COM 2 Selected", m_com2Stby); return true;
    case BaseDeviceHandler::VriCommand::ComS:        m_com2Stby->updateVri2Xp(command.m_value);         return true;
    case BaseDeviceHandler::VriCommand::ComX:        transferRadio("COM2 exchange", m_com2Act, m_com2Stby);    return true;

    case BaseDeviceHandler::VriCommand::NavSel1:    selectRadio("Radio Navigation 1 Selected", m_nav1Stby); return true;
    case BaseDeviceHandler::VriCommand::Navs:        m_nav1Stby->updateVri2Xp(command.m_value);     return true;
    case BaseDeviceHandler::VriCommand::Navx:        transferRadio("Radio Navigation 1 exchange", m_nav1Act, m_nav1Stby);    return true;

    case BaseDeviceHandler::VriCommand::NavSel2:    selectRadio("Radio Navigation 2 Selected", m_nav2Stby); return true;
    case BaseDeviceHandler::VriCommand::NavS:        m_nav2Stby->updateVri2Xp(command.m_value);         return true;
    case BaseDeviceHandler::VriCommand::NavX:        transferRadio("Radio Navigation 2 exchange", m_nav2Act, m_nav2Stby);    return true;

    case BaseDeviceHandler::VriCommand::DmeSel1:    m_radioMode = RadioDisplay::RadioMode::Dme1; return true;
    case BaseDeviceHandler::VriCommand::DmeSel2:    m_radioMode = RadioDisplay::RadioMode::Dme2; return true;


    case BaseDeviceHandler::VriCommand::CrsxxxMin:
    case BaseDeviceHandler::VriCommand::CrsxxxPlus:

        switch (m_radioMode) {
        case RadioDisplay::RadioMode::Dme1:
            m_dme1->update(command.m_value);
            break;

        case RadioDisplay::RadioMode::Dme2:
            m_dme2->update(command.m_value);
            break;

        default:
            break;
        }
        return true;



    case BaseDeviceHandler::VriCommand::TrnSel:        selectRadio("Transponder Selected", m_transponder);        return true;
    case BaseDeviceHandler::VriCommand::Trns:        m_transponder->updateVri2Xp(command.m_value);        return true;

    case BaseDeviceHandler::VriCommand::TrnAux:
        scheduleCommand(BaseDeviceHandler::VriCommand::TrnAux);
        return true;

    case BaseDeviceHandler::VriCommand::Trnx:
        scheduleCommand(BaseDeviceHandler::VriCommand::Trnx);
        return true;



    case BaseDeviceHandler::VriCommand::AplAtPlus:
        if (getDatai(BaseDeviceHandler::VriDataRef::DR_AutoThrottle, -1) != 1)
        {
            scheduleWithSpeechCommand(command.m_command, "Auto Throttle Enabled");
        }
        return true;

    case BaseDeviceHandler::VriCommand::AplAtMin:

        if (getDatai(BaseDeviceHandler::VriDataRef::DR_AutoThrottle, -1) != 0) {
            scheduleWithSpeechCommand(command.m_command, "Auto Throttle Disabled");
        }
        return true;

    case BaseDeviceHandler::VriCommand::AplFdPlus:

        if (getDatai(BaseDeviceHandler::VriDataRef::DR_FlightDirector, -1) != 1) {
            scheduleWithSpeechCommand(command.m_command, "Flight Director Enabled");
        }
        return true;

    case BaseDeviceHandler::VriCommand::AplFdMin:

        if (getDatai(BaseDeviceHandler::VriDataRef::DR_FlightDirector, -1) != 0) {
            scheduleWithSpeechCommand(command.m_command, "Flight Director Disabled");
        }
        return true;

        // reverse
    case BaseDeviceHandler::VriCommand::AplMastPlus:

        if (getDatai(BaseDeviceHandler::VriDataRef::DR_ApMaster, -1) != 0) {
            scheduleWithSpeechCommand(command.m_command, "Autopilot Master Disabled");
        }
        return true;

    case BaseDeviceHandler::VriCommand::AplMastMin: // is disable

        if (getDatai(BaseDeviceHandler::VriDataRef::DR_ApMaster, -1) != 1) {
            scheduleWithSpeechCommand(command.m_command, "Autopilot Master Enabled");
        }
        return true;



    case BaseDeviceHandler::VriCommand::HdgSelPlus:
        Speak("BankAngle Selected");
        hdgUpperSelected = true;
        bankAngle = updateBankAngle();
        return true;

    case BaseDeviceHandler::VriCommand::HdgSelMin:
        Speak("Heading Selected");
        hdgUpperSelected = false;
        return true;

    case BaseDeviceHandler::VriCommand::HdgXXXPlus:
        if (hdgUpperSelected) {
            return  handleBankAngle(command.m_command);
        }

        scheduleCommand(command.m_command);
        return true;

    case BaseDeviceHandler::VriCommand::HdgXXXMin:
        if (hdgUpperSelected) {
            return  handleBankAngle(command.m_command);
        }

        scheduleCommand(command.m_command);
        return true;

        // MIN

    case BaseDeviceHandler::VriCommand::MinSelPlus:

        if (checkFastToggle(minSelectedMs, 200)) {
            scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand::vMinReset, "Minimums Reset");
            return true;
        }

        Speak("Minimums Upper Selected");
        minUpperSelected = true;
        return true;

    case BaseDeviceHandler::VriCommand::MinSelMin:

        if (checkFastToggle(minSelectedMs, 200)) {
            scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand::vMinReset, "Minimums Reset");
            return true;
        }

        Speak("Minimums Lower Selected");

        minUpperSelected = false;
        return true;

    case BaseDeviceHandler::VriCommand::MinPlus:

        if (minUpperSelected) {
            scheduleCommand(BaseDeviceHandler::VriCommand::vMinUpperPlus);
            return true;

        }
        scheduleCommand(BaseDeviceHandler::VriCommand::vMinLowerPlus);
        return true;

    case BaseDeviceHandler::VriCommand::MinMin:

        if (minUpperSelected) {
            scheduleCommand(BaseDeviceHandler::VriCommand::vMinUpperMin);

            return true;
        }

        scheduleCommand(BaseDeviceHandler::VriCommand::vMinLowerMin);
        return true;


        // BARO
    case BaseDeviceHandler::VriCommand::BarSelPlus:
        if (checkFastToggle(baroSelectedMs, 200)) {
            scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand::vBarStd, "Baro Standard");
            return true;
        }

        Speak("Baro Upper Selected");
        baroUpperSelected = true;
        return true;

    case BaseDeviceHandler::VriCommand::BarSelMin:

        if (checkFastToggle(baroSelectedMs, 200)) {
            scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand::vBarStd, "Baro Standard");
            return true;
        }

        Speak("Baro Lower Selected");
        baroUpperSelected = false;

    case BaseDeviceHandler::VriCommand::BarPlus:

        if (baroUpperSelected) {
            scheduleCommand(BaseDeviceHandler::VriCommand::vBarUpperPlus);
            return true;

        }
        scheduleCommand(BaseDeviceHandler::VriCommand::vBarLowerPlus);
        return true;

    case BaseDeviceHandler::VriCommand::BarMin:

        if (baroUpperSelected) {
            scheduleCommand(BaseDeviceHandler::VriCommand::vBarUpperMin);
            return true;
        }

        scheduleCommand(BaseDeviceHandler::VriCommand::vBarLowerMin);
        return true;

    case BaseDeviceHandler::VriCommand::NdrSelMin:
    case BaseDeviceHandler::VriCommand::NdrSelPlus:
        scheduleWithSpeechCommand(command.m_command, "TFC Selected");
        break;

    case BaseDeviceHandler::VriCommand::NdrMin:
        setMapRange(command.m_command);
        return true;

    case BaseDeviceHandler::VriCommand::NdrPlus:
        setMapRange(command.m_command);
        return true;

    case BaseDeviceHandler::VriCommand::NdmSelMin: //CTR
    case BaseDeviceHandler::VriCommand::NdmSelPlus:
        scheduleWithSpeechCommand(command.m_command, "CTR Selected");
        return true;


    case BaseDeviceHandler::VriCommand::NdmPlus:
        setMapMode(command.m_command);
        return true;
    case BaseDeviceHandler::VriCommand::NdmMin:
        setMapMode(command.m_command);
        return true;



    case BaseDeviceHandler::VriCommand::EfiVor1:        setEfiVor(BaseDeviceHandler::VriDataRef::DR_Vor1Pos, command.m_command); return true;
    case BaseDeviceHandler::VriCommand::EfiAdf1:        setEfiAdf(BaseDeviceHandler::VriDataRef::DR_Vor1Pos, command.m_command); return true;

    case BaseDeviceHandler::VriCommand::EfiVor2:        setEfiVor(BaseDeviceHandler::VriDataRef::DR_Vor2Pos, command.m_command); return true;
    case BaseDeviceHandler::VriCommand::EfiAdf2:        setEfiAdf(BaseDeviceHandler::VriDataRef::DR_Vor2Pos, command.m_command); return true;

    case BaseDeviceHandler::VriCommand::AplTogaPlus:    scheduleWithSpeechCommand(command.m_command, "Left Take Off And Go Arround");    return true;
    case BaseDeviceHandler::VriCommand::AplTogaMin:        scheduleWithSpeechCommand(command.m_command, "Left Take Off And Go Arround");    return true;

    case BaseDeviceHandler::VriCommand::AplTogbPlus:    scheduleWithSpeechCommand(command.m_command, "Right Take Off And Go Arround");    return true;
    case BaseDeviceHandler::VriCommand::AplTogbMin:        scheduleWithSpeechCommand(command.m_command, "Right Take Off And Go Arround");    return true;

    case BaseDeviceHandler::VriCommand::OBSPlus:        setCourse(command.m_command);         return true;
    case BaseDeviceHandler::VriCommand::OBSMin:            setCourse(command.m_command);         return true;

    case BaseDeviceHandler::VriCommand::OBSSelPlus:        syncCourse();return true;
    case BaseDeviceHandler::VriCommand::OBSSelMin:        syncCourse();return true;


    case BaseDeviceHandler::VriCommand::EfiMtr:        scheduleWithSpeechCommand(command.m_command, "Meters");                        return true;
    case BaseDeviceHandler::VriCommand::EfiFpv:        scheduleWithSpeechCommand(command.m_command, "flight path vector");    return true;
    case BaseDeviceHandler::VriCommand::EfiWx:        scheduleWithSpeechCommand(command.m_command, "Weather");            return true;
    case BaseDeviceHandler::VriCommand::EfiSta:        scheduleWithSpeechCommand(command.m_command, "Station");            return true;
    case BaseDeviceHandler::VriCommand::EfiWpt:        scheduleWithSpeechCommand(command.m_command, "Waypoints");            return true;
    case BaseDeviceHandler::VriCommand::EfiArpt:    scheduleWithSpeechCommand(command.m_command, "Airports");            return true;
    case BaseDeviceHandler::VriCommand::EfiData:    scheduleWithSpeechCommand(command.m_command, "Data");                return true;
    case BaseDeviceHandler::VriCommand::EfiPos:        scheduleWithSpeechCommand(command.m_command, "Position");            return true;
    case BaseDeviceHandler::VriCommand::EfiTerr:    scheduleWithSpeechCommand(command.m_command, "Terrain");            return true;

    case BaseDeviceHandler::VriCommand::SpdSelPlus: scheduleWithSpeechCommand(command.m_command, "Speed Pressed");        return true;
    case BaseDeviceHandler::VriCommand::SpdSelMin:  scheduleWithSpeechCommand(command.m_command, "Speed Pressed");        return true;
        // speed
    case BaseDeviceHandler::VriCommand::SpdXXXPlus:
    case BaseDeviceHandler::VriCommand::SpdXXXMin:
        scheduleCommand(command.m_command);
        return true;

    case BaseDeviceHandler::VriCommand::SpdN1Plus:
    case BaseDeviceHandler::VriCommand::SpdN1Min:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_SpdN1, command.m_command, "N1 Select", "DSP0N1  ", NO_IDENT1);
        return true;


    case BaseDeviceHandler::VriCommand::SpdSpdPlus:
    case BaseDeviceHandler::VriCommand::SpdSpdMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_SpdSpd, command.m_command, "Speed Select", "DSP0SPD ", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::SpdLvlPlus:
    case BaseDeviceHandler::VriCommand::SpdLvlMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_SpdLvl, command.m_command, "FlightLevel change", "DSP0FLCH", NO_IDENT1);
        return true;

        // heading

    case BaseDeviceHandler::VriCommand::HdgHdgPlus:
    case BaseDeviceHandler::VriCommand::HdgHdgMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_HdgHdg, command.m_command, "Heading Select", "DSP0HDG ", "DSP1SEL ");
        return true;

    case BaseDeviceHandler::VriCommand::HdgHldPlus:
    case BaseDeviceHandler::VriCommand::HdgHldMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_HdgHld, command.m_command, "Heading Hold Select", "DSP0HDG ", "DSP1HLD ");
        return true;

        // altitude
    case BaseDeviceHandler::VriCommand::AltXXXPlus:
    case BaseDeviceHandler::VriCommand::AltXXXMin:
        scheduleCommand(command.m_command);
        return true;

    case BaseDeviceHandler::VriCommand::AltHldPlus:
    case BaseDeviceHandler::VriCommand::AltHldMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AltHld, command.m_command, "Altitude Hold Select", "DSP0ALT ", "DSP1HLD ");
        return true;

        //vvs
    case BaseDeviceHandler::VriCommand::VvsPlus:
    case BaseDeviceHandler::VriCommand::VvsMin:
        setVvs(command.m_command);
        return true;

    case BaseDeviceHandler::VriCommand::VvsSelPlus:
    case BaseDeviceHandler::VriCommand::VvsSelMin:
        resetVvs();
        return true;
        

    case BaseDeviceHandler::VriCommand::VvsHldPlus:
    case BaseDeviceHandler::VriCommand::VvsHldMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_VvsHld, command.m_command, "Vertical Speed Select", "DSP0V/S ", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::AplVnavPlus:
    case BaseDeviceHandler::VriCommand::AplVnavMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplVnav, command.m_command, "VNAV", "DSP0VNAV", NO_IDENT1);

        return true;

    case BaseDeviceHandler::VriCommand::AplLnavPlus:
    case BaseDeviceHandler::VriCommand::AplLnavMin:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplLnav, command.m_command, "LNAV", "DSP0LNAV", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::AplCmdAMin:
    case BaseDeviceHandler::VriCommand::AplCmdAPlus:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplCmdA, command.m_command, "Command A", "DSP0CMDA", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::AplCmdBMin:
    case BaseDeviceHandler::VriCommand::AplCmdBPlus:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplCmdB, command.m_command, "Command B", "DSP0CMDB", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::AplCmdCMin:

        if (commandDefined(command.m_command)) {
            scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplCmdC, command.m_command, "Command C", "DSP0CMDC", NO_IDENT1);
            return true;
        }
        setLights();
        return true;

    case BaseDeviceHandler::VriCommand::AplCmdCPlus:

        if (commandDefined(command.m_command)) {
            scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplCmdC, command.m_command, "Command C", "DSP0CMDC", NO_IDENT1);
            return true;
        }
        setLights();
        return true;

    case BaseDeviceHandler::VriCommand::AplLocMin:
    case BaseDeviceHandler::VriCommand::AplLocPlus:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplLoc, command.m_command, "LOC", "DSP0LOC ", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::AplCwsAMin:
    case BaseDeviceHandler::VriCommand::AplCwsAPlus:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_CwsA, command.m_command, "CWS A", "DSP0CWSA ", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::AplCwsBMin:
    case BaseDeviceHandler::VriCommand::AplCwsBPlus:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_CwsB, command.m_command, "CWS B", "DSP0CWSB ", NO_IDENT1);
        return true;

    case BaseDeviceHandler::VriCommand::AplAppMin:
    case BaseDeviceHandler::VriCommand::AplAppPlus:
        scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef::DR_AplApp, command.m_command, "Approach", "DSP0APP ", NO_IDENT1);
        return true;



    case BaseDeviceHandler::VriCommand::CtlBn0On:scheduleWithSpeechCommand(command.m_command, "landing lights"); return true;
    case BaseDeviceHandler::VriCommand::CtlBn0Of:scheduleWithSpeechCommand(command.m_command, "landing lights"); return true;

    case BaseDeviceHandler::VriCommand::CtlBn1On:
        scheduleWithSpeechCommand(command.m_command, "taxi lights",2);
        return true;
    case BaseDeviceHandler::VriCommand::CtlBn1Of:
        scheduleWithSpeechCommand(command.m_command, "taxi lights", 2);
        return true;

    case BaseDeviceHandler::VriCommand::CtlBn2On:scheduleWithSpeechCommand(command.m_command, "logo lights"); return true;
    case BaseDeviceHandler::VriCommand::CtlBn2Of:scheduleWithSpeechCommand(command.m_command, "logo lights"); return true;

    case BaseDeviceHandler::VriCommand::CtlBn3On:scheduleWithSpeechCommand(command.m_command, "wing lights"); return true;
    case BaseDeviceHandler::VriCommand::CtlBn3Of:scheduleWithSpeechCommand(command.m_command, "wing lights"); return true;

    case BaseDeviceHandler::VriCommand::CtlBn4On:scheduleWithSpeechCommand(command.m_command, "wheel lights"); return true;
    case BaseDeviceHandler::VriCommand::CtlBn4Of:scheduleWithSpeechCommand(command.m_command, "weel lights"); return true;

    case BaseDeviceHandler::VriCommand::CtlBn5On:scheduleWithSpeechCommand(command.m_command, "APU"); return true;
    case BaseDeviceHandler::VriCommand::CtlBn5Of:scheduleWithSpeechCommand(command.m_command, "APU"); return true;

    case BaseDeviceHandler::VriCommand::CtlBn6On:    scheduleWithSpeechCommand(command.m_command, "Landing gear");         return true;
    case BaseDeviceHandler::VriCommand::CtlBn6Of:    scheduleWithSpeechCommand(command.m_command, "Landing gear");         return true;

    case BaseDeviceHandler::VriCommand::CtlBn7On:
    case BaseDeviceHandler::VriCommand::CtlBn7Of:

        if (commandDefined(command.m_command)) {
            scheduleWithSpeechCommand(command.m_command, nullptr);
        }
        if (speechActive) {
            XPLMSpeakString("Speech disabled");
            updateIdent("DSP0no");
            updateIdent("DSP1spch");

        }
        else {

            XPLMSpeakString("Speech enabled");
            updateIdent("DSP0spch");
            updateIdent("DSP1actv");
        }

        speechActive = !speechActive;
        return true;

    default:
        Speak("Unassigned button detected");

        return true;
    }

    return true;


}

void BaseAircraft::selectRadio(const char* radio, RadioDisplay* stby) {

    m_radioMode = stby->getRadioMode();

    updateRadios();


}


bool BaseAircraft::transferRadio(const char* radio, RadioDisplay* act, RadioDisplay* stby) {

    Speak(radio);

    int activeFreq = act->getFrequency();
    int stbyFreq = stby->getFrequency();


    stby->updateXp(activeFreq);
    act->updateXp(stbyFreq);

    int newActiveFreq = act->getFrequency();
    int newStbyFreq = stby->getFrequency();
    VLLog("transferRadio %s Active %d StandBy %d  <=x=> Active %d StandBy %d", radio, activeFreq, stbyFreq, newActiveFreq, newStbyFreq);

    updateRadios();

    return true;

}


bool BaseAircraft::commandDefined(BaseDeviceHandler::VriCommand vriCommand) {

    XPLMCommandRef command;
    bool ret = getXPLMCommandRef(vriCommand, command);
    if (!ret) {
        VLTrace("commandDefined: vriCommand [%d] not defined", vriCommand);
    }
    return ret;

}

bool BaseAircraft::dataRefDefined(BaseDeviceHandler::VriDataRef vriDataRef) {

    XPLMDataRef ref;
    bool ret = getXPLMDataRef(vriDataRef, ref);
    if (!ret) {
        VLError("dataRefDefined: vriDataRef [%d] not defined");
    }
    return ret;

}

bool BaseAircraft::handleBankAngle(BaseDeviceHandler::VriCommand command) {

    if (command == BaseDeviceHandler::VriCommand::HdgXXXPlus) {
        // BANK ANGLE        0 - 10deg, 1 - 15deg, 2 - 20deg, 3 - 25deg, 4 - 30deg

        if (bankAngle < 4) {
            bankAngle++;
            setDatai(BaseDeviceHandler::VriDataRef::DR_BankAngle, bankAngle);
        }

    }
    else {
        if (bankAngle > 0) {
            bankAngle--;
            setDatai(BaseDeviceHandler::VriDataRef::DR_BankAngle, bankAngle);
        }


    }
    bankAngle = updateBankAngle();

    return true;
}

int BaseAircraft::updateBankAngle()
    {


    int value = getDatai(BaseDeviceHandler::VriDataRef::DR_BankAngle, -1);

    updateIdent("DSP0BANK");

    switch (value) {
    case 0:        Speak("BankAngle 10 degrees");        updateIdent("DSP110dg");        break;
    case 1:        Speak("BankAngle 15 degrees");        updateIdent("DSP115dg");        break;
    case 2:        Speak("BankAngle 20 degrees");        updateIdent("DSP120dg");        break;
    case 3:        Speak("BankAngle 25 degrees");        updateIdent("DSP125dg");        break;
    case 4:        Speak("BankAngle 30 degrees");        updateIdent("DSP130dg");        break;
    default:
        break;
    }

    m_speed->updateDisplay();

    
    return value;

    
}

bool BaseAircraft::scheduleCommand(BaseDeviceHandler::VriCommand VriCommand, int repeat)
{
    if (VriCommand == BaseDeviceHandler::VriCommand::None) {
        VLError("scheduleCommand: command None ignored");
        return false;
    }

    XPLMCommandRef command;
    bool ret = getXPLMCommandRef(VriCommand, command);
    if (!ret) {
        VLError("scheduleCommand: command not found");
        return false;
    }
    if (command == 0)
    {
        VLLog("Not scheduling empty command reference");
        return false;
    }

    std::lock_guard<std::mutex> guard(commandMutex);
    if (m_xpCommands == nullptr)
        m_xpCommands = new std::queue<XPLMCommandRef>();

    m_xpCommands->push(command);
    if (repeat > 1)
    {
        for (int r = 0;r < repeat - 1;r++)
            m_xpCommands->push(command);
    }

    return true;

}

void BaseAircraft::setEfiAdf(BaseDeviceHandler::VriDataRef vriDataRef, BaseDeviceHandler::VriCommand VriCommand) {

    scheduleCommand(VriCommand);

    int status = getDatai(vriDataRef, 0);

    switch (status) {
    case 0: Speak("Adf selected");
        
        break;
    case 1: Speak("VOR deselected"); break;
    case -1://already selected
        return;

    default:
        break;
    }


}

void BaseAircraft::setEfiVor(BaseDeviceHandler::VriDataRef vriDataRef, BaseDeviceHandler::VriCommand VriCommand) {

    int status = getDatai(vriDataRef, 0);

    switch (status) {
    case 0: Speak("VOR selected"); break;
    case -1: Speak("Adf deselected"); break;
    case 1: //already selected
        return;

    default:
        break;
    }

    scheduleCommand(VriCommand);

}

void BaseAircraft::resetVvs() {
    VLLog("resetVvs");
    Speak("Reset vertical speed to 0");

    setDataf(BaseDeviceHandler::VriDataRef::DR_Vvs, 0);

    updateIdent("DSP0VS R");
    updateIdent("DSP1ESET");

}

void BaseAircraft::displayVvs() {
    
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    float vvs = getDataf(BaseDeviceHandler::VriDataRef::DR_Vvs, 0);
    char command[MAX_DISPLAY]{ 0 };
    

    if (vvs < 0) {
        updateIdent("DSP0VSdn");
        sprintf(command, "DSP1%04d", (int) -vvs);

    }
    else {
        updateIdent("DSP0VSup");
        sprintf(command, "DSP1%04d", (int) vvs);

    }
    updateIdent(command);

}


void BaseAircraft::setVvs(BaseDeviceHandler::VriCommand VriCommand) {
    scheduleCommand(VriCommand);
    displayVvs();
    
}
void BaseAircraft::syncCourse() {
    

    float course = m_heading->getActualXpValue();
    VLLog("syncCourse: %3.f", course);
    setDataf(BaseDeviceHandler::VriDataRef::DR_CoursePilot, course);
    Speak("Course Pilot synced to heading ");
    char command[MAX_DISPLAY]{ 0 };
    sprintf(command, "DSP1 %03d", (int)course);

    updateIdent("DSP0CRS ");
    updateIdent(command);
}


void BaseAircraft::setCourse(BaseDeviceHandler::VriCommand vriCommand) {

    scheduleCommand(vriCommand);


    bool ret = dataRefDefined(BaseDeviceHandler::VriDataRef::DR_Course);

    if (ret) {
        float course = getDataf(BaseDeviceHandler::VriDataRef::DR_Course, 0);

        switch (vriCommand) {
        case BaseDeviceHandler::VriCommand::OBSPlus:
            course++;
            if (course > 359) {
                course = 0;
            }
            break;

        case BaseDeviceHandler::VriCommand::OBSMin:
            course--;
            if (course < 0) {
                course = 359;
            }
            break;

        default:
            break;

        }

        char command[MAX_DISPLAY]{ 0 };
        sprintf(command, "DSP1 %03d", (int) course);

        updateIdent("DSP0CRS ");
        updateIdent(command);
    }






}



void BaseAircraft::setMapMode(BaseDeviceHandler::VriCommand VriCommand) {



    scheduleCommand(VriCommand);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    int status = getDatai(BaseDeviceHandler::VriDataRef::DR_MapMode, 0);


    // Map mode. 0 = approach, 1 = vor, 2 = map, 3 = nav, 4 = plan"
    switch (status) {
    case 0: Speak("App selected");        break;
    case 1: Speak("VOR selected"); break;
    case 2: Speak("MAP selected"); break;
    case 3: Speak("Plan selected"); break;

    default:
        break;
    }


}

void BaseAircraft::setMapRange(BaseDeviceHandler::VriCommand VriCommand) {

    scheduleCommand(VriCommand);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    int status = getDatai(BaseDeviceHandler::VriDataRef::DR_MapRange, 0);

    switch (status) {
    case 0: Speak("Map Range 5 selected");        break;
    case 1: Speak("Map Range 10 selected"); break;
    case 2: Speak("Map Range 20 selected"); break;
    case 3: Speak("Map Range 40 selected"); break;
    case 4: Speak("Map Range 80 selected"); break;
    case 5: Speak("Map Range 160 selected"); break;
    case 6: Speak("Map Range 320 selected"); break;
    case 7: Speak("Map Range 640 selected"); break;
        return;

    default:
        break;
    }



}


void BaseAircraft::setLights() {
    switch (lightMode) {
    case 0:
        scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand::AplDomeOn, "dome light on", 2);
        updateIdent("DSP0DOME");
        updateIdent("DSP1 on ");
        break;

    case 1:
        scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand::AplDomeOff, "dome light off", 2);
        updateIdent("DSP0DOME");
        updateIdent("DSP1 off");
        break;

    case 2:
        setPanelLight(1, 1, 1, 1);
        setGenericLight(1, 1, 1, 1, 1,1);
        updateIdent("DSP0Lght");
        updateIdent("DSP1   1");
        Speak("Light Mode 1");
        break;

    case 3:
        setPanelLight(1, 1, 1, 1);
        setGenericLight(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
        updateIdent("DSP0Lght");
        updateIdent("DSP1   2");
        Speak("Light Mode 2");
        break;

    case 4:
        setPanelLight(1, 1, 1, 1);
        setGenericLight(0, 0, 0, 0, 0, 0);
        updateIdent("DSP0Lght");
        updateIdent("DSP1   3");
        Speak("Light Mode 3");
        break;

    case 5:
        setPanelLight(0.5, 0.5, 0.5, 0.5);
        setGenericLight(0, 0, 0, 0, 0, 0);
        updateIdent("DSP0Lght");
        updateIdent("DSP1   4");
        Speak("Light Mode 4");

        break;

    case 6:
        setPanelLight(0.5, 0.5, 0.5, 0.5);
        setGenericLight(0.2, 0.2, 0.0, 0.2, 0.2, 0.2);
        updateIdent("DSP0Lght");
        updateIdent("DSP1   5");
        Speak("Light Mode 5");

        break;

    default:
        setPanelLight(0, 0, 0, 0);
        setGenericLight(0, 0, 0, 1, 0, 0);
        lightMode = 0;
        updateIdent(NO_IDENT0);
        updateIdent(NO_IDENT1);
        Speak("Lights out");

        return;

    }

    lightMode++;
}
void BaseAircraft::Speak(const char* speech) {


    if (speech == nullptr) {
        return;
    }
    if (!speechActive) {
        return;
    }

    XPLMSpeakString(speech);

}
bool BaseAircraft::scheduleWithSpeechCommand(BaseDeviceHandler::VriCommand VriCommand, const char* speech, int repeat)
{
    if (VriCommand == BaseDeviceHandler::VriCommand::None) {
        VLError("scheduleCommand: command None ignored");
        return false;
    }

    Speak(speech);


    return scheduleCommand(VriCommand, repeat);

}
bool BaseAircraft::scheduleButtonCommandWithDisplay(BaseDeviceHandler::VriDataRef vriDataRef, BaseDeviceHandler::VriCommand VriCommand,
    const char* speech, const char* ident0, const char* ident1, int repeat)
{

    if (VriCommand == BaseDeviceHandler::VriCommand::None) {
        VLError("scheduleCommand: command None ignored");
        return false;
    }


    if (speechActive) {
        char message[64];

        int status = getDatai(vriDataRef, -1);


        if (status != 1) {
            sprintf(message, "%s %s", speech, "activated");
            updateIdent(ident0);
            updateIdent(ident1);

        }
        else {
            sprintf(message, "%s %s", speech, "deactivated");
            updateIdent("DSP0    ");
            updateIdent(NO_IDENT1);
        }

        Speak(message);
    }

    return scheduleCommand(VriCommand, repeat);

}
bool BaseAircraft::scheduleButtonCommand(BaseDeviceHandler::VriDataRef vriDataRef, BaseDeviceHandler::VriCommand VriCommand,
    const char* speech, int repeat)
{

    if (VriCommand == BaseDeviceHandler::VriCommand::None) {
        VLError("scheduleCommand: command None ignored");
        return false;
    }


    if (speechActive) {
        char message[64];

        int status = getDatai(vriDataRef, -1);


        if (status != 1) {
            sprintf(message, "%s %s", speech, "activated");
        }
        else {
            sprintf(message, "%s %s", speech, "deactivated");
        }

        Speak(message);
    }

    return scheduleCommand(VriCommand, repeat);

}

CommandQueue* BaseAircraft::queuedCommands()
{
    std::queue<XPLMCommandRef>* returnedCommands;
    commandMutex.lock();
    returnedCommands = m_xpCommands;
    m_xpCommands = nullptr;
    commandMutex.unlock();
    return returnedCommands;
}
bool BaseAircraft::isLoaded()
{
    return isPlaneConfigured;
}



bool BaseAircraft::getXPLMCommandRef(BaseDeviceHandler::VriCommand VriCommand, XPLMCommandRef& result) {

    if (cmdMap.count(VriCommand) != 0)
    {
        result = cmdMap[VriCommand];
        return true;
    }
    return false;

}

bool BaseAircraft::getXPLMDataRef(BaseDeviceHandler::VriDataRef vriDataRef, XPLMDataRef& result) {

    if (drMap.count(vriDataRef) != 0)
    {
        result = drMap[vriDataRef];
        return true;
    }

    VLError("getXPLMDataRef: cannot detect %d ", vriDataRef);
    return false;

}

float BaseAircraft::getDataf(BaseDeviceHandler::VriDataRef vriDataRef, float defValue) {

    XPLMDataRef ref;
    bool ret = getXPLMDataRef(vriDataRef, ref);
    if (!ret) {
        return defValue;
    }

    return XPLMGetDataf(ref);

}
void BaseAircraft::setDataf(BaseDeviceHandler::VriDataRef vriDataRef, float value) {

    XPLMDataRef ref;
    bool ret = getXPLMDataRef(vriDataRef, ref);
    if (!ret) {
        return;
    }

    XPLMSetDataf(ref, value);

}
void BaseAircraft::setDatai(BaseDeviceHandler::VriDataRef vriDataRef, int value) {

    XPLMDataRef ref;
    bool ret = getXPLMDataRef(vriDataRef, ref);
    if (!ret) {
        return;
    }

    XPLMSetDatai(ref, value);

}
int BaseAircraft::getDatai(BaseDeviceHandler::VriDataRef vriDataRef, int defValue) {

    XPLMDataRef ref;
    bool ret = getXPLMDataRef(vriDataRef, ref);
    if (!ret) {
        return defValue;
    }

    return XPLMGetDatai(ref);

}


bool BaseAircraft::getDatab(BaseDeviceHandler::VriDataRef vriDataRef, void* data, int max) {

    XPLMDataRef ref;
    bool ret = getXPLMDataRef(vriDataRef, ref);
    if (!ret) {
        return false;
    }

    XPLMGetDatab(ref, data, 0, max);
    return true;
}

bool BaseAircraft::scanForPlane()
{

    updateIdent(NO_IDENT0);
    updateIdent(NO_IDENT1);

    VLLog("scanForPlane: Looking for current User Plane based on tailnum");

    storeDataRef(BaseDeviceHandler::VriDataRef::DR_TailNum, "sim/aircraft/view/acf_tailnum");

    char tailNum[MAX_TAIL_NR] = { 0 };

    bool ret = getDatab(BaseDeviceHandler::VriDataRef::DR_TailNum, tailNum, MAX_TAIL_NR);
    if (!ret) {
        VLError("scanForPlane: tailNum not available");
        return false;
    }

    ret = readConfigFile(tailNum);
    if (!ret) {
        VLError("scanForPlane: configuration file not available");
        XPLMSpeakString("Error in COMBO config file detected");


        if (!startsWith(tailNum, "ZB")) {
            return false;
        }

        initData();
        XPLMSpeakString("Using default ZIBO settings");


    }

    VLLog("Welcome %s", tailNum);

    

    initMcpDisplays();
    initRadios();
    
    updateMcpDisplays();
    initializeRadios();

    isPlaneConfigured = true;


    updateIdent("DSP0ZIBO");
    updateIdent("DSP1v1.6");

    return true;


}

bool BaseAircraft::readConfigFile(const char* tailNum) {

    VLLog("readConfigFile: Loading [%s]", tailNum);

    char configFile[255] = { 0 };
    sprintf(configFile, "%s%sConfig.csv", PLUGIN_CONFIG_PATH, tailNum);

    VLLog("readConfigFile: Loading configFile [%s]", configFile);
    std::ifstream inFile(configFile);

    if (!inFile.is_open())
    {
        VLError("readConfigFile: Can't open configFile [%s]", configFile);
        return false;
    }
    string lineStr;

    cmdMap.clear();
    drMap.clear();

    VLLog("readConfigFile: cmdMap size [%d]", cmdMap.size());
    VLLog("readConfigFile: drMap size [%d]", drMap.size());


    int lineCnt = 0;

    bool ret = true;
    while (getline(inFile, lineStr))
    {
        lineCnt++;
        if (startsWith(lineStr, "#")) {
            continue;
        }

        if (!contains(lineStr, ',')) {
            continue;
        }

        //  Split string
        size_t index = lineStr.find(",");
        if (index <= 0) {
            continue;
        }

        try {
            string commandOrRef = lineStr.substr(0, index);
            string xplaneCommandOrDataref = lineStr.substr(index + 1, lineStr.size() - 1);
            if (startsWith(xplaneCommandOrDataref, "INTERNAL_USE_ONLY")) {
                continue;
            }

            char* writable = new char[xplaneCommandOrDataref.size() + 1];
            std::copy(xplaneCommandOrDataref.begin(), xplaneCommandOrDataref.end(), writable);
            writable[xplaneCommandOrDataref.size()] = '\0'; // don't forget the terminating 0


            if (startsWith(lineStr, "DR_")) {
                // dataref
                BaseDeviceHandler::VriDataRef dataRef = ConvertToVriDataref(commandOrRef);
                if (dataRef == BaseDeviceHandler::VriDataRef::DR_None) {
                    VLError("readConfigFile: line [%d] VriDataRef [%s] not mapped", configFile, lineCnt, commandOrRef.c_str());
                    continue;
                }
                ret &= storeDataRef(dataRef, writable);
                continue;
            }

            // dataref
            BaseDeviceHandler::VriCommand vriCommand = ConvertToVriCommand(commandOrRef);
            if (vriCommand == BaseDeviceHandler::VriCommand::None) {
                VLError("readConfigFile: line [%d] VriCommand [%s] not mapped", configFile, lineCnt, commandOrRef.c_str());
                continue;
            }
            ret &= storeCommand(vriCommand, writable);
            continue;
        }
        catch (int e) {
            VLError("readConfigFile: line [%d] incorrect. error [%d]", configFile, lineCnt, e);
        }

    }



    return true;
}

bool BaseAircraft::startsWith(const std::string& str, const std::string& cmp)
{
    return str.compare(0, cmp.length(), cmp) == 0;
}
bool BaseAircraft::contains(const std::string& str, char cmp)
{
    if (str.find(cmp) != string::npos) {
        return true;
    }

    return false;

}

string BaseAircraft::TrimFunction(string str)
{
    const char* typeOfWhitespaces = " tnrfv";
    str.erase(str.find_last_not_of(typeOfWhitespaces) + 1);
    str.erase(0, str.find_first_not_of(typeOfWhitespaces));
    return str;
}

BaseDeviceHandler::VriDataRef BaseAircraft::ConvertToVriDataref(string vriDataRef) {

    if (startsWith(vriDataRef, "DR_Altitude")) return BaseDeviceHandler::VriDataRef::DR_Altitude;
    if (startsWith(vriDataRef, "DR_CoursePilot")) return BaseDeviceHandler::VriDataRef::DR_CoursePilot;
    if (startsWith(vriDataRef, "DR_Speed")) return BaseDeviceHandler::VriDataRef::DR_Speed;
    if (startsWith(vriDataRef, "DR_Heading")) return BaseDeviceHandler::VriDataRef::DR_Heading;
    if (startsWith(vriDataRef, "DR_Vvs")) return BaseDeviceHandler::VriDataRef::DR_Vvs;

    if (startsWith(vriDataRef, "DR_Com1Standby")) return BaseDeviceHandler::VriDataRef::DR_Com1Standby;
    if (startsWith(vriDataRef, "DR_Com1")) return BaseDeviceHandler::VriDataRef::DR_Com1;

    if (startsWith(vriDataRef, "DR_Com2Standby")) return BaseDeviceHandler::VriDataRef::DR_Com2Standby;
    if (startsWith(vriDataRef, "DR_Com2")) return BaseDeviceHandler::VriDataRef::DR_Com2;

    if (startsWith(vriDataRef, "DR_Nav1Standby")) return BaseDeviceHandler::VriDataRef::DR_Nav1Standby;
    if (startsWith(vriDataRef, "DR_Nav1")) return BaseDeviceHandler::VriDataRef::DR_Nav1;

    if (startsWith(vriDataRef, "DR_Nav2Standby")) return BaseDeviceHandler::VriDataRef::DR_Nav2Standby;
    if (startsWith(vriDataRef, "DR_Nav2")) return BaseDeviceHandler::VriDataRef::DR_Nav2;

    if (startsWith(vriDataRef, "DR_Dme1Dist")) return BaseDeviceHandler::VriDataRef::DR_Dme1Dist;
    if (startsWith(vriDataRef, "DR_Dme1Ident")) return BaseDeviceHandler::VriDataRef::DR_Dme1Ident;
    if (startsWith(vriDataRef, "DR_Dme1Speed")) return BaseDeviceHandler::VriDataRef::DR_Dme1Speed;
    if (startsWith(vriDataRef, "DR_Dme1Course")) return BaseDeviceHandler::VriDataRef::DR_Dme1Course;

    if (startsWith(vriDataRef, "DR_Dme2Dist")) return BaseDeviceHandler::VriDataRef::DR_Dme2Dist;
    if (startsWith(vriDataRef, "DR_Dme2Ident")) return BaseDeviceHandler::VriDataRef::DR_Dme2Ident;
    if (startsWith(vriDataRef, "DR_Dme2Speed")) return BaseDeviceHandler::VriDataRef::DR_Dme2Speed;
    if (startsWith(vriDataRef, "DR_Dme2Course")) return BaseDeviceHandler::VriDataRef::DR_Dme2Course;

    if (startsWith(vriDataRef, "DR_TransponderCode")) return BaseDeviceHandler::VriDataRef::DR_TransponderCode;
    if (startsWith(vriDataRef, "DR_TransponderMode")) return BaseDeviceHandler::VriDataRef::DR_TransponderMode;

    if (startsWith(vriDataRef, "DR_AutoThrottle")) return BaseDeviceHandler::VriDataRef::DR_AutoThrottle;
    if (startsWith(vriDataRef, "DR_FlightDirector")) return BaseDeviceHandler::VriDataRef::DR_FlightDirector;

    if (startsWith(vriDataRef, "DR_ApMaster")) return BaseDeviceHandler::VriDataRef::DR_ApMaster;

    if (startsWith(vriDataRef, "DR_SpdN1")) return BaseDeviceHandler::VriDataRef::DR_SpdN1;
    if (startsWith(vriDataRef, "DR_SpdSpd")) return BaseDeviceHandler::VriDataRef::DR_SpdSpd;
    if (startsWith(vriDataRef, "DR_SpdLvl")) return BaseDeviceHandler::VriDataRef::DR_SpdLvl;

    if (startsWith(vriDataRef, "DR_HdgHdg")) return BaseDeviceHandler::VriDataRef::DR_HdgHdg;
    if (startsWith(vriDataRef, "DR_HdgHld"))return BaseDeviceHandler::VriDataRef::DR_HdgHld;
    if (startsWith(vriDataRef, "DR_BankAngle")) return BaseDeviceHandler::VriDataRef::DR_BankAngle;

    if (startsWith(vriDataRef, "DR_AltHld")) return BaseDeviceHandler::VriDataRef::DR_AltHld;
    if (startsWith(vriDataRef, "DR_VvsHld")) return BaseDeviceHandler::VriDataRef::DR_VvsHld;
    if (startsWith(vriDataRef, "DR_AplVnav")) return BaseDeviceHandler::VriDataRef::DR_AplVnav;
    if (startsWith(vriDataRef, "DR_AplLnav")) return BaseDeviceHandler::VriDataRef::DR_AplLnav;
    if (startsWith(vriDataRef, "DR_AplCmdA")) return BaseDeviceHandler::VriDataRef::DR_AplCmdA;
    if (startsWith(vriDataRef, "DR_AplCmdB")) return BaseDeviceHandler::VriDataRef::DR_AplCmdB;
    if (startsWith(vriDataRef, "DR_AplCmdC")) return BaseDeviceHandler::VriDataRef::DR_AplCmdC;
    if (startsWith(vriDataRef, "DR_CwsA")) return BaseDeviceHandler::VriDataRef::DR_CwsA;
    if (startsWith(vriDataRef, "DR_CwsB")) return BaseDeviceHandler::VriDataRef::DR_CwsB;
    if (startsWith(vriDataRef, "DR_AplApp")) return BaseDeviceHandler::VriDataRef::DR_AplApp;
    if (startsWith(vriDataRef, "DR_AplLoc")) return BaseDeviceHandler::VriDataRef::DR_AplLoc;

    if (startsWith(vriDataRef, "DR_Adf1Standby")) return BaseDeviceHandler::VriDataRef::DR_Adf1Standby;
    if (startsWith(vriDataRef, "DR_Adf1")) return BaseDeviceHandler::VriDataRef::DR_Adf1;
    if (startsWith(vriDataRef, "DR_Adf2Standby")) return BaseDeviceHandler::VriDataRef::DR_Adf2Standby;
    if (startsWith(vriDataRef, "DR_Adf2")) return BaseDeviceHandler::VriDataRef::DR_Adf2;

    if (startsWith(vriDataRef, "DR_PanelBrightness")) return BaseDeviceHandler::VriDataRef::DR_PanelBrightness;
    if (startsWith(vriDataRef, "DR_GenericLightsSwitch")) return BaseDeviceHandler::VriDataRef::DR_GenericLightsSwitch;
    if (startsWith(vriDataRef, "DR_MapMode")) return BaseDeviceHandler::VriDataRef::DR_MapMode;
    if (startsWith(vriDataRef, "DR_MapRange")) return BaseDeviceHandler::VriDataRef::DR_MapRange;
    if (startsWith(vriDataRef, "DR_Vor1Pos")) return BaseDeviceHandler::VriDataRef::DR_Vor1Pos;
    if (startsWith(vriDataRef, "DR_Vor2Pos")) return BaseDeviceHandler::VriDataRef::DR_Vor2Pos;
    if (startsWith(vriDataRef, "DR_Course")) return BaseDeviceHandler::VriDataRef::DR_Course;

    return BaseDeviceHandler::VriDataRef::DR_None;

}

BaseDeviceHandler::VriCommand BaseAircraft::ConvertToVriCommand(string VriCommand) {

    if (startsWith(VriCommand, "AdfSel1")) return BaseDeviceHandler::VriCommand::AdfSel1;
    if (startsWith(VriCommand, "AdfSel2")) return BaseDeviceHandler::VriCommand::AdfSel2;
    if (startsWith(VriCommand, "AdfAux")) return BaseDeviceHandler::VriCommand::AdfAux;

    if (startsWith(VriCommand, "Adf")) return BaseDeviceHandler::VriCommand::Adf;

    if (startsWith(VriCommand, "AltHldMin")) return BaseDeviceHandler::VriCommand::AltHldMin;
    if (startsWith(VriCommand, "AltHldPlus")) return BaseDeviceHandler::VriCommand::AltHldPlus;
    if (startsWith(VriCommand, "AltSelMin")) return BaseDeviceHandler::VriCommand::AltSelMin;
    if (startsWith(VriCommand, "AltSelPlus")) return BaseDeviceHandler::VriCommand::AltSelPlus;
    if (startsWith(VriCommand, "AltXXXMin")) return BaseDeviceHandler::VriCommand::AltXXXMin;
    if (startsWith(VriCommand, "AltXXXPlus")) return BaseDeviceHandler::VriCommand::AltXXXPlus;

    if (startsWith(VriCommand, "AplAppMin")) return BaseDeviceHandler::VriCommand::AplAppMin;
    if (startsWith(VriCommand, "AplAppPlus")) return BaseDeviceHandler::VriCommand::AplAppPlus;
    if (startsWith(VriCommand, "AplAtMin")) return BaseDeviceHandler::VriCommand::AplAtMin;
    if (startsWith(VriCommand, "AplAtPlus")) return BaseDeviceHandler::VriCommand::AplAtPlus;
    if (startsWith(VriCommand, "AplCmdAMin")) return BaseDeviceHandler::VriCommand::AplCmdAMin;
    if (startsWith(VriCommand, "AplCmdAPlus")) return BaseDeviceHandler::VriCommand::AplCmdAPlus;
    if (startsWith(VriCommand, "AplCmdBMin")) return BaseDeviceHandler::VriCommand::AplCmdBMin;
    if (startsWith(VriCommand, "AplCmdBPlus")) return BaseDeviceHandler::VriCommand::AplCmdBPlus;
    if (startsWith(VriCommand, "AplCmdCMin")) return BaseDeviceHandler::VriCommand::AplCmdCMin;
    if (startsWith(VriCommand, "AplCmdCPlus")) return BaseDeviceHandler::VriCommand::AplCmdCPlus;
    if (startsWith(VriCommand, "AplDomeOn")) return BaseDeviceHandler::VriCommand::AplDomeOn;
    if (startsWith(VriCommand, "AplDomeOff")) return BaseDeviceHandler::VriCommand::AplDomeOff;


    if (startsWith(VriCommand, "AplCwsAMin")) return BaseDeviceHandler::VriCommand::AplCwsAMin;
    if (startsWith(VriCommand, "AplCwsAPlus")) return BaseDeviceHandler::VriCommand::AplCwsAPlus;
    if (startsWith(VriCommand, "AplCwsBMin")) return BaseDeviceHandler::VriCommand::AplCwsBMin;
    if (startsWith(VriCommand, "AplCwsBPlus")) return BaseDeviceHandler::VriCommand::AplCwsBPlus;

    if (startsWith(VriCommand, "AplFdMin")) return BaseDeviceHandler::VriCommand::AplFdMin;
    if (startsWith(VriCommand, "AplFdPlus")) return BaseDeviceHandler::VriCommand::AplFdPlus;

    if (startsWith(VriCommand, "AplLnavMin")) return BaseDeviceHandler::VriCommand::AplLnavMin;
    if (startsWith(VriCommand, "AplLnavPlus")) return BaseDeviceHandler::VriCommand::AplLnavPlus;

    if (startsWith(VriCommand, "AplLocMin")) return BaseDeviceHandler::VriCommand::AplLocMin;
    if (startsWith(VriCommand, "AplLocPlus")) return BaseDeviceHandler::VriCommand::AplLocPlus;

    if (startsWith(VriCommand, "AplMastMin")) return BaseDeviceHandler::VriCommand::AplMastMin;
    if (startsWith(VriCommand, "AplMastPlus")) return BaseDeviceHandler::VriCommand::AplMastPlus;

    if (startsWith(VriCommand, "AplTogaMin")) return BaseDeviceHandler::VriCommand::AplTogaMin;
    if (startsWith(VriCommand, "AplTogaPlus")) return BaseDeviceHandler::VriCommand::AplTogaPlus;

    if (startsWith(VriCommand, "AplTogbMin")) return BaseDeviceHandler::VriCommand::AplTogbMin;
    if (startsWith(VriCommand, "AplTogbPlus")) return BaseDeviceHandler::VriCommand::AplTogbPlus;

    if (startsWith(VriCommand, "AplVnavMin")) return BaseDeviceHandler::VriCommand::AplVnavMin;
    if (startsWith(VriCommand, "AplVnavPlus")) return BaseDeviceHandler::VriCommand::AplVnavPlus;

    if (startsWith(VriCommand, "BankAngleMin")) return BaseDeviceHandler::VriCommand::BankAngleMin;
    if (startsWith(VriCommand, "BankAnglePlus")) return BaseDeviceHandler::VriCommand::BankAnglePlus;

    if (startsWith(VriCommand, "vBarLowerMin")) return BaseDeviceHandler::VriCommand::vBarLowerMin;
    if (startsWith(VriCommand, "vBarLowerPlus")) return BaseDeviceHandler::VriCommand::vBarLowerPlus;
    if (startsWith(VriCommand, "vBarUpperMin")) return BaseDeviceHandler::VriCommand::vBarUpperMin;
    if (startsWith(VriCommand, "vBarUpperPlus")) return BaseDeviceHandler::VriCommand::vBarUpperPlus;
    if (startsWith(VriCommand, "vBarStd")) return BaseDeviceHandler::VriCommand::vBarStd;
    if (startsWith(VriCommand, "BarSelMin")) return BaseDeviceHandler::VriCommand::BarSelMin;
    if (startsWith(VriCommand, "BarSelPlus")) return BaseDeviceHandler::VriCommand::BarSelPlus;

    if (startsWith(VriCommand, "ComAux")) return BaseDeviceHandler::VriCommand::ComAux;
    if (startsWith(VriCommand, "ComSel1")) return BaseDeviceHandler::VriCommand::ComSel1;
    if (startsWith(VriCommand, "ComSel2")) return BaseDeviceHandler::VriCommand::ComSel2;
    if (startsWith(VriCommand, "Coms")) return BaseDeviceHandler::VriCommand::Coms;
    if (startsWith(VriCommand, "ComS")) return BaseDeviceHandler::VriCommand::ComS;
    if (startsWith(VriCommand, "Comx")) return BaseDeviceHandler::VriCommand::Comx;
    if (startsWith(VriCommand, "ComX")) return BaseDeviceHandler::VriCommand::ComX;

    if (startsWith(VriCommand, "CrsxxxMin")) return BaseDeviceHandler::VriCommand::CrsxxxMin;
    if (startsWith(VriCommand, "CrsxxxPlus")) return BaseDeviceHandler::VriCommand::CrsxxxPlus;
    if (startsWith(VriCommand, "Crs")) return BaseDeviceHandler::VriCommand::Crs;

    if (startsWith(VriCommand, "CtlBn0Of")) return BaseDeviceHandler::VriCommand::CtlBn0Of;
    if (startsWith(VriCommand, "CtlBn0On")) return BaseDeviceHandler::VriCommand::CtlBn0On;
    if (startsWith(VriCommand, "CtlBn1Of")) return BaseDeviceHandler::VriCommand::CtlBn1Of;
    if (startsWith(VriCommand, "CtlBn1On")) return BaseDeviceHandler::VriCommand::CtlBn1On;
    if (startsWith(VriCommand, "CtlBn2Of")) return BaseDeviceHandler::VriCommand::CtlBn2Of;
    if (startsWith(VriCommand, "CtlBn2On")) return BaseDeviceHandler::VriCommand::CtlBn2On;
    if (startsWith(VriCommand, "CtlBn3Of")) return BaseDeviceHandler::VriCommand::CtlBn3Of;
    if (startsWith(VriCommand, "CtlBn3On")) return BaseDeviceHandler::VriCommand::CtlBn3On;
    if (startsWith(VriCommand, "CtlBn4Of")) return BaseDeviceHandler::VriCommand::CtlBn4Of;
    if (startsWith(VriCommand, "CtlBn4On")) return BaseDeviceHandler::VriCommand::CtlBn4On;
    if (startsWith(VriCommand, "CtlBn5Of")) return BaseDeviceHandler::VriCommand::CtlBn5Of;
    if (startsWith(VriCommand, "CtlBn5On")) return BaseDeviceHandler::VriCommand::CtlBn5On;
    if (startsWith(VriCommand, "CtlBn6Of")) return BaseDeviceHandler::VriCommand::CtlBn6Of;
    if (startsWith(VriCommand, "CtlBn6On")) return BaseDeviceHandler::VriCommand::CtlBn6On;
    if (startsWith(VriCommand, "CtlBn7Of")) return BaseDeviceHandler::VriCommand::CtlBn7Of;
    if (startsWith(VriCommand, "CtlBn7On")) return BaseDeviceHandler::VriCommand::CtlBn7On;

    if (startsWith(VriCommand, "DmeAux")) return BaseDeviceHandler::VriCommand::DmeAux;
    if (startsWith(VriCommand, "DmeSel1")) return BaseDeviceHandler::VriCommand::DmeSel1;
    if (startsWith(VriCommand, "DmeSel2")) return BaseDeviceHandler::VriCommand::DmeSel2;

    if (startsWith(VriCommand, "EfiAdf1")) return BaseDeviceHandler::VriCommand::EfiAdf1;
    if (startsWith(VriCommand, "EfiAdf2")) return BaseDeviceHandler::VriCommand::EfiAdf2;
    if (startsWith(VriCommand, "EfiArpt")) return BaseDeviceHandler::VriCommand::EfiArpt;
    if (startsWith(VriCommand, "EfiData")) return BaseDeviceHandler::VriCommand::EfiData;
    if (startsWith(VriCommand, "EfiFpv")) return BaseDeviceHandler::VriCommand::EfiFpv;
    if (startsWith(VriCommand, "EfiMtr")) return BaseDeviceHandler::VriCommand::EfiMtr;
    if (startsWith(VriCommand, "EfiPos")) return BaseDeviceHandler::VriCommand::EfiPos;
    if (startsWith(VriCommand, "EfiSta")) return BaseDeviceHandler::VriCommand::EfiSta;
    if (startsWith(VriCommand, "EfiTerr")) return BaseDeviceHandler::VriCommand::EfiTerr;
    if (startsWith(VriCommand, "EfiVor1")) return BaseDeviceHandler::VriCommand::EfiVor1;
    if (startsWith(VriCommand, "EfiVor2")) return BaseDeviceHandler::VriCommand::EfiVor2;
    if (startsWith(VriCommand, "EfiWpt")) return BaseDeviceHandler::VriCommand::EfiWpt;
    if (startsWith(VriCommand, "EfiWx")) return BaseDeviceHandler::VriCommand::EfiWx;

    if (startsWith(VriCommand, "HdgHdgMin")) return BaseDeviceHandler::VriCommand::HdgHdgMin;
    if (startsWith(VriCommand, "HdgHdgPlus")) return BaseDeviceHandler::VriCommand::HdgHdgPlus;
    if (startsWith(VriCommand, "HdgHldMin")) return BaseDeviceHandler::VriCommand::HdgHldMin;
    if (startsWith(VriCommand, "HdgHldPlus")) return BaseDeviceHandler::VriCommand::HdgHldPlus;
    if (startsWith(VriCommand, "HdgSelMin")) return BaseDeviceHandler::VriCommand::HdgSelMin;
    if (startsWith(VriCommand, "HdgSelPlus")) return BaseDeviceHandler::VriCommand::HdgSelPlus;
    if (startsWith(VriCommand, "HdgXXXMin")) return BaseDeviceHandler::VriCommand::HdgXXXMin;
    if (startsWith(VriCommand, "HdgXXXPlus")) return BaseDeviceHandler::VriCommand::HdgXXXPlus;

    if (startsWith(VriCommand, "vMinLowerPlus")) return BaseDeviceHandler::VriCommand::vMinLowerPlus;
    if (startsWith(VriCommand, "vMinLowerMin")) return BaseDeviceHandler::VriCommand::vMinLowerMin;
    if (startsWith(VriCommand, "vMinUpperMin")) return BaseDeviceHandler::VriCommand::vMinUpperMin;
    if (startsWith(VriCommand, "vMinUpperPlus")) return BaseDeviceHandler::VriCommand::vMinUpperPlus;
    if (startsWith(VriCommand, "vMinReset")) return BaseDeviceHandler::VriCommand::vMinReset;

    if (startsWith(VriCommand, "MinSelPlus")) return BaseDeviceHandler::VriCommand::MinSelPlus;
    if (startsWith(VriCommand, "MinSelMin")) return BaseDeviceHandler::VriCommand::MinSelMin;

    if (startsWith(VriCommand, "NavAux")) return BaseDeviceHandler::VriCommand::NavAux;
    if (startsWith(VriCommand, "NavSel1")) return BaseDeviceHandler::VriCommand::NavSel1;
    if (startsWith(VriCommand, "NavSel2")) return BaseDeviceHandler::VriCommand::NavSel2;
    if (startsWith(VriCommand, "Navs")) return BaseDeviceHandler::VriCommand::Navs;
    if (startsWith(VriCommand, "NavS")) return BaseDeviceHandler::VriCommand::NavS;
    if (startsWith(VriCommand, "Navx")) return BaseDeviceHandler::VriCommand::Navx;
    if (startsWith(VriCommand, "NavX")) return BaseDeviceHandler::VriCommand::NavX;

    if (startsWith(VriCommand, "NdmMin")) return BaseDeviceHandler::VriCommand::NdmMin;
    if (startsWith(VriCommand, "NdmPlus")) return BaseDeviceHandler::VriCommand::NdmPlus;
    if (startsWith(VriCommand, "NdmSelMin")) return BaseDeviceHandler::VriCommand::NdmSelMin;
    if (startsWith(VriCommand, "NdmSelPlus")) return BaseDeviceHandler::VriCommand::NdmSelPlus;

    if (startsWith(VriCommand, "NdrMin")) return BaseDeviceHandler::VriCommand::NdrMin;
    if (startsWith(VriCommand, "NdrPlus")) return BaseDeviceHandler::VriCommand::NdrPlus;
    if (startsWith(VriCommand, "NdrSelMin")) return BaseDeviceHandler::VriCommand::NdrSelMin;
    if (startsWith(VriCommand, "NdrSelPlus")) return BaseDeviceHandler::VriCommand::NdrSelPlus;

    if (startsWith(VriCommand, "OBSMin")) return BaseDeviceHandler::VriCommand::OBSMin;
    if (startsWith(VriCommand, "OBSPlus")) return BaseDeviceHandler::VriCommand::OBSPlus;
    if (startsWith(VriCommand, "OBSSelMin")) return BaseDeviceHandler::VriCommand::OBSSelMin;
    if (startsWith(VriCommand, "OBSSelPlus")) return BaseDeviceHandler::VriCommand::OBSSelPlus;

    if (startsWith(VriCommand, "SpdLvlMin")) return BaseDeviceHandler::VriCommand::SpdLvlMin;
    if (startsWith(VriCommand, "SpdLvlPlus")) return BaseDeviceHandler::VriCommand::SpdLvlPlus;
    if (startsWith(VriCommand, "SpdN1Min")) return BaseDeviceHandler::VriCommand::SpdN1Min;
    if (startsWith(VriCommand, "SpdN1Plus")) return BaseDeviceHandler::VriCommand::SpdN1Plus;
    if (startsWith(VriCommand, "SpdSelMin")) return BaseDeviceHandler::VriCommand::SpdSelMin;
    if (startsWith(VriCommand, "SpdSelPlus")) return BaseDeviceHandler::VriCommand::SpdSelPlus;
    if (startsWith(VriCommand, "SpdSpdMin")) return BaseDeviceHandler::VriCommand::SpdSpdMin;
    if (startsWith(VriCommand, "SpdSpdPlus")) return BaseDeviceHandler::VriCommand::SpdSpdPlus;
    if (startsWith(VriCommand, "SpdXXXMin")) return BaseDeviceHandler::VriCommand::SpdXXXMin;
    if (startsWith(VriCommand, "SpdXXXPlus")) return BaseDeviceHandler::VriCommand::SpdXXXPlus;

    if (startsWith(VriCommand, "TrnAux")) return BaseDeviceHandler::VriCommand::TrnAux;
    if (startsWith(VriCommand, "Trns")) return BaseDeviceHandler::VriCommand::Trns;
    if (startsWith(VriCommand, "TrnSel")) return BaseDeviceHandler::VriCommand::TrnSel;
    if (startsWith(VriCommand, "Trnx")) return BaseDeviceHandler::VriCommand::Trnx;

    if (startsWith(VriCommand, "VvsHldMin")) return BaseDeviceHandler::VriCommand::VvsHldMin;
    if (startsWith(VriCommand, "VvsHldPlus")) return BaseDeviceHandler::VriCommand::VvsHldPlus;
    if (startsWith(VriCommand, "VvsMin")) return BaseDeviceHandler::VriCommand::VvsMin;
    if (startsWith(VriCommand, "VvsPlus")) return BaseDeviceHandler::VriCommand::VvsPlus;
    if (startsWith(VriCommand, "VvsSelMin")) return BaseDeviceHandler::VriCommand::VvsSelMin;
    if (startsWith(VriCommand, "VvsSelPlus")) return BaseDeviceHandler::VriCommand::VvsSelPlus;



    return BaseDeviceHandler::VriCommand::None;

}




void BaseAircraft::setDatavf(BaseDeviceHandler::VriDataRef vriDataRef, float* inValues, int inoffset, int inCount) {

    XPLMDataRef ref;
    bool ret = getXPLMDataRef(vriDataRef, ref);
    if (!ret) {
        return;
    }

    XPLMSetDatavf(ref, inValues, inoffset, inCount);

}


void BaseAircraft::setPanelLight(float captMainPanel, float foMainPanel, float overheadPanel, float pedestalPanel) {

    float panel[4];
    panel[0] = captMainPanel;
    panel[1] = foMainPanel;
    panel[2] = overheadPanel;
    panel[3] = pedestalPanel;

    setDatavf(BaseDeviceHandler::VriDataRef::DR_PanelBrightness, panel, 0, 4);
}

void BaseAircraft::setGenericLight(    float forwardPanelFlood, float glareShieldFlood, float pedestalFlood,
        float captChartLight, float foChartLight, float circuitBrakerFlood) {

    float genricLight[13];
    genricLight[6] = forwardPanelFlood;
    genricLight[7] = glareShieldFlood;
    genricLight[8] = pedestalFlood;
    genricLight[10] = captChartLight;
    genricLight[11] = foChartLight;
    genricLight[12] = circuitBrakerFlood;
    setDatavf(BaseDeviceHandler::VriDataRef::DR_GenericLightsSwitch, genricLight, 0, 12);

}





bool BaseAircraft::checkFastToggle(uint64_t& last, uint64_t interval) {
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    uint64_t diff = ms - last;

    last = ms;

    if (diff < interval) {
        return true;
    }
    return false;


}





