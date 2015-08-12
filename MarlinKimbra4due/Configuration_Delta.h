// Define this to set a custom name for your generic Delta
#define CUSTOM_MENDEL_NAME "Delta"

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================

// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#define DELTA_SEGMENTS_PER_SECOND 200

// Center-to-center distance of the holes in the diagonal push rods.
//#define DEFAULT_DELTA_DIAGONAL_ROD 208.0      // mm
#define DEFAULT_DELTA_DIAGONAL_ROD 190.0      // mm


// Horizontal offset from middle of printer to smooth rod center.
//#define DELTA_SMOOTH_ROD_OFFSET 127.1      // mm
#define DELTA_SMOOTH_ROD_OFFSET 132.0      // mm


// Horizontal offset of the universal joints on the end effector.
//#define DELTA_EFFECTOR_OFFSET 10.0      // mm
#define DELTA_EFFECTOR_OFFSET 25.0      // mm


// Horizontal offset of the universal joints on the carriages.
//#define DELTA_CARRIAGE_OFFSET 10.0      // mm
#define DELTA_CARRIAGE_OFFSET 9.75      // mm


// Bed Printer radius
//#define PRINTER_RADIUS 70     // mm
#define PRINTER_RADIUS 88     // mm


//Uncomment to enable autocalibration debug messages
#define DEBUG_MESSAGES

// Precision for G30 delta autocalibration function
#define AUTOCALIBRATION_PRECISION 0.05      // mm

// Effective horizontal distance bridged by diagonal push rods.
#define DEFAULT_DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)

// Z-Probe variables
// Start and end location values are used to deploy/retract the probe (will move from start to end and back again)
#define PROBING_FEEDRATE 500                     // Speed for individual probe Use: G30 A F600
#define Z_PROBE_OFFSET {0,0,-1,0}                // X, Y, Z, E distance between hotend nozzle and deployed bed leveling probe.
#define Z_PROBE_DEPLOY_START_LOCATION {0,0,50,0} // X, Y, Z, E start location for z-probe deployment sequence
#define Z_PROBE_DEPLOY_END_LOCATION {0,0,50,0}   // X, Y, Z, E end location for z-probe deployment sequence
#define Z_PROBE_RETRACT_START_LOCATION {0,0,50,0}// X, Y, Z, E start location for z-probe retract sequence
#define Z_PROBE_RETRACT_END_LOCATION {0,0,50,0}  // X, Y, Z, E end location for z-probe retract sequence
#define AUTOLEVEL_GRID 20                        // Distance between autolevel Z probing points, should be less than print surface radius/3.

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// coarse Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
  // #define ENDSTOPPULLUP_ZMAX
  // #define ENDSTOPPULLUP_XMIN
  // #define ENDSTOPPULLUP_YMIN
  // #define ENDSTOPPULLUP_ZMIN
  // #define ENDSTOPPULLUP_ZPROBE
  // #define ENDSTOPPULLUP_EMIN
#endif

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
const bool X_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool Y_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.
const bool Z_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.

const bool E_MIN_ENDSTOP_INVERTING = false;     // set to true to invert the logic of the endstop.


const bool X_MAX_ENDSTOP_INVERTING = true;     // set to true to invert the logic of the endstop.
const bool Y_MAX_ENDSTOP_INVERTING = true;     // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = true;     // set to true to invert the logic of the endstop.
//aven_0414_2015 Enable z_probe_endstop start
//const bool Z_PROBE_ENDSTOP_INVERTING = false;   // set to true to invert the logic of the endstop.
//const bool Z_PROBE_ENDSTOP_INVERTING = true;
//aven_0414_2015 Enable z_probe_endstop end

//const bool X_MAX_ENDSTOP_INVERTING = false;
//const bool Y_MAX_ENDSTOP_INVERTING = false;
//const bool Z_MAX_ENDSTOP_INVERTING = false;
const bool Z_PROBE_ENDSTOP_INVERTING = false;


// If you want to enable the Z Probe pin, but disable its use, uncomment the line below.
// This only affects a Z Probe Endstop if you have separate Z min endstop as well and have
// activated Z_PROBE_ENDSTOP below. If you are using the Z Min endstop on your Z Probe,
// this has no effect.
//#define DISABLE_Z_PROBE_ENDSTOP

// ENDSTOP SETTINGS:
// Sets direction of endstop when homing; 1=MAX, -1=MIN
#define X_HOME_DIR 1 //DELTA MUST HAVE MAX ENDSTOP
#define Y_HOME_DIR 1 //DELTA MUST HAVE MAX ENDSTOP
#define Z_HOME_DIR 1 //DELTA MUST HAVE MAX ENDSTOP

//aven_0813
//#define min_software_endstops true  // If true, axis won't move to coordinates less than HOME_POS.
//#define max_software_endstops true  // If true, axis won't move to coordinates greater than the defined lengths below.

#define min_software_endstops false  // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops false


// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0      // For all extruder

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false      // For all extruder
#define DISABLE_INACTIVE_EXTRUDER true //disable only inactive extruder and keep active extruder enabled

// If you motor turns to wrong direction, you can invert it here:
#define INVERT_X_DIR false
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
//#define INVERT_E0_DIR false
//aven_0501
#define INVERT_E0_DIR false

#define INVERT_E1_DIR false
#define INVERT_E2_DIR false
#define INVERT_E3_DIR false

// The position of the homing switches
#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0      // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS 240//205      // Distance between nozzle and print surface after homing.

// Travel limits after homing (units are in mm)
#define X_MAX_POS PRINTER_RADIUS
#define X_MIN_POS -PRINTER_RADIUS
#define Y_MAX_POS PRINTER_RADIUS
#define Y_MIN_POS -PRINTER_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS
#define Z_MIN_POS 0
#define E_MIN_POS 0

//aven_0414_2015 Enable z_probe_endstop start
#define Z_PROBE_ENDSTOP
//aven_0414_2015 Enable z_probe_endstop end


// MOVEMENT SETTINGS
#define HOMING_FEEDRATE {100*60, 100*60, 100*60, 0}      // set the homing speeds (mm/min)

// default settings
// delta speeds must be the same on xyz
//#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,80,95,625,625,625}        // X, Y, Z, E0, E1, E2, E3
//#define DEFAULT_AXIS_STEPS_PER_UNIT {160,160,160,95,50,625,625}        // X, Y, Z, E0, E1, E2, E3

//aven 0716
//#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,80,95,50,625,625}        // X, Y, Z, E0, E1, E2, E3

//aven 0727
#define DEFAULT_AXIS_STEPS_PER_UNIT {80,80,80,95,25.6,625,625}  // X, Y, Z, E0, E1, E2, E3
//#define DEFAULT_AXIS_STEPS_PER_UNIT {160,160,160,95,50,625,625} 


#define DEFAULT_MAX_FEEDRATE {300,300,300,200,100,100,100}             // X, Y, Z, E0, E1, E2, E3 (mm/sec)
#define DEFAULT_RETRACTION_MAX_FEEDRATE {150,150,150,150}             // E0, E1, E2, E3 (mm/sec)
#define DEFAULT_MAX_ACCELERATION {2000,2000,2000,1000,1000,1000,1000} // X, Y, Z, E0, E1, E2, E3 maximum start speed for accelerated moves.

#define DEFAULT_ACCELERATION          1000      // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  2500      // X, Y, Z and E max acceleration in mm/s^2 for retracts
#define DEFAULT_TRAVEL_ACCELERATION   3000      // X, Y, Z acceleration in mm/s^2 for travel (non printing) moves

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
//#define HOTEND_OFFSET_X {0.0, 5.00, 0.0, 0.0} // (in mm) for each extruder, offset of the hotend on the X axis
//#define HOTEND_OFFSET_Y {0.0, 5.00, 0.0, 0.0} // (in mm) for each extruder, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK 20  // (mm/sec)
#define DEFAULT_ZJERK  20  // (mm/sec)
#define DEFAULT_EJERK   5  // (mm/sec)
