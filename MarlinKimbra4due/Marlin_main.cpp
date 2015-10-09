/* -*- c++ -*- */

/*
 Reprap firmware based on Marlin
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 Copyright (C) 2014 MagoKimbra

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Marlin.h"

//aven_0812
#include "vector_3.h"


#ifdef ENABLE_AUTO_BED_LEVELING
  #include "vector_3.h"
  #ifdef AUTO_BED_LEVELING_GRID
    #include "qr_solve.h"
  #endif
#endif // ENABLE_AUTO_BED_LEVELING

#define SERVO_LEVELING (defined(ENABLE_AUTO_BED_LEVELING) && PROBE_SERVO_DEACTIVATION_DELAY > 0)


#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "motion_control.h"
#include "cardreader.h"
#include "watchdog.h"
#include "ConfigurationStore.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"

#ifdef BLINKM
  #include "BlinkM.h"
  #include "Wire.h"
#endif

#if NUM_SERVOS > 0
  #include "Servo.h"
#endif

#if HAS_DIGIPOTSS
  #include <SPI.h>
#endif

#ifdef FIRMWARE_TEST
  #include "firmware_test.h"
#endif

// look here for descriptions of G-codes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

/* Implemented Codes
-------------------
G0  -> G1
G1  - Coordinated Movement X Y Z E
G2  - CW ARC
G3  - CCW ARC
G4  - Dwell S<seconds> or P<milliseconds>
G10 - retract filament according to settings of M207
G11 - retract recover filament according to settings of M208
G28 - Home all Axis
G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
G30 - Single Z Probe, probes bed at current XY location. - Bed Probe and Delta geometry Autocalibration
G31 - Dock sled (Z_PROBE_SLED only)
G32 - Undock sled (Z_PROBE_SLED only)
G60 - Store in memory actual position
G61 - Move X Y Z to position in memory
G90 - Use Absolute Coordinates
G91 - Use Relative Coordinates
G92 - Set current position to coordinates given

M Codes
M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
M1   - Same as M0
M03  - Put S<value> in laser beam control
M04  - Turn on laser beam
M05  - Turn off laser beam
M11  - Start printer for pause mode
M17  - Enable/Power all stepper motors
M18  - Disable all stepper motors; same as M84
M20  - List SD card
M21  - Init SD card
M22  - Release SD card
M23  - Select SD file (M23 filename.g)
M24  - Start/resume SD print
M25  - Pause SD print
M26  - Set SD position in bytes (M26 S12345)
M27  - Report SD print status
M28  - Start SD write (M28 filename.g)
M29  - Stop SD write
M30  - Delete file from SD (M30 filename.g)
M31  - Output time since last M109 or SD card start to serial
M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
       syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
       Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
       The '#' is necessary when calling from within sd files, as it stops buffer prereading
M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
M49  - Z probe repetability test
M80  - Turn on Power Supply
M81  - Turn off Power, including Power Supply, if possible
M82  - Set E codes absolute (default)
M83  - Set E codes relative while in Absolute Coordinates (G90) mode
M84  - Disable steppers until next move,
       or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
M92  - Set axis_steps_per_unit - same syntax as G92
M104 - Set extruder target temp
M105 - Read current temp
M106 - Fan on
M107 - Fan off
M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
       Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
       IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
M111 - Debug mode
M112 - Emergency stop
M114 - Output current position to serial port
M115 - Capabilities string
M117 - display message
M119 - Output Endstop status to serial port
M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
M140 - Set bed target temp
M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
M200 D<millimeters>- set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000 Z1000 E0 S1000 E1 S1000 E2 S1000 E3 S1000).
M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E0 S1000 E1 S1000 E2 S1000 E3 S1000) in mm/sec
M204 - Set Accelerations in mm/sec^2: S printing moves, R Retract moves(only E), T travel moves (M204 P1200 R3000 T2500) im mm/sec^2  also sets minimum segment time in ms (B20000) to prevent buffer underruns and M20 minimum feedrate
M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
M206 - set additional homing offset
M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
M208 - set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/sec]
M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
M218 - set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
M220 S<factor in percent>- set speed factor override percentage
M221 S<factor in percent>- set extrude factor override percentage
M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
M240 - Trigger a camera to take a photograph
M250 - Set LCD contrast C<contrast value> (value 0..63)
M280 - set servo position absolute. P: servo index, S: angle or microseconds
M300 - Play beep sound S<frequency Hz> P<duration ms>
M301 - Set PID parameters P I and D
M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
M304 - Set bed PID parameters P I and D
M350 - Set microstepping mode.
M351 - Toggle MS1 MS2 pins directly.
M380 - Activate solenoid on active extruder
M381 - Disable all solenoids
M400 - Finish all moves
M401 - Lower z-probe if present
M402 - Raise z-probe if present
M404 - D<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder 
M406 - Turn off Filament Sensor extrusion control 
M407 - Displays measured filament diameter 
M500 - Store parameters in EEPROM
M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).
M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
M503 - Print the current settings (from memory not from EEPROM). Use S0 to leave off headings.
M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
M666 - Set z probe offset or Endstop and delta geometry adjustment
M907 - Set digital trimpot motor current using axis codes.
M908 - Control digital trimpot directly.

************ SCARA Specific - This can change to suit future G-code regulations
M360 - SCARA calibration: Move to calc-position ThetaA (0 deg calibration)
M361 - SCARA calibration: Move to calc-position ThetaB (90 deg calibration - steps per degree)
M362 - SCARA calibration: Move to calc-position PsiA (0 deg calibration)
M363 - SCARA calibration: Move to calc-position PsiB (90 deg calibration - steps per degree)
M364 - SCARA calibration: Move to calc-position PSIC (90 deg to Theta calibration position)
M365 - SCARA calibration: Scaling factor, X, Y, Z axis
************* SCARA End ***************

M928 - Start SD logging (M928 filename.g) - ended by M29
M997 - NPR2 Color rotate
M999 - Restart after being stopped by error
*/

#ifdef SDSUPPORT
  CardReader card;
#endif

float homing_feedrate[] = HOMING_FEEDRATE;
int homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;
int feedmultiply = 100; //100->1 200->2
int saved_feedmultiply;
int extruder_multiply[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100, 100, 100, 100);
bool volumetric_enabled = false;
float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA, DEFAULT_NOMINAL_FILAMENT_DIA);
float volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0, 1.0, 1.0, 1.0);
float current_position[NUM_AXIS] = { 0.0 };
float destination[NUM_AXIS] = { 0.0 };
float lastpos[NUM_AXIS] = { 0.0 };
float home_offset[3] = { 0 };
float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = { false };
uint8_t active_extruder = 0;
uint8_t active_driver = 0;
uint8_t debugLevel = 0;
int fanSpeed = 0;
bool cancel_heatup = false;
const char errormagic[] PROGMEM = "Error:";
const char echomagic[] PROGMEM = "echo:";
const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
static float offset[3] = { 0 };
static float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
static char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
static int bufindr = 0;
static int bufindw = 0;
static int buflen = 0;
static char serial_char;
static int serial_count = 0;
// static boolean comment_mode = false; // Use in get_command() only and can be removed.
static char *strchr_pointer; ///< A pointer to find chars in the command string (X, Y, Z, E, etc.)
const char* queued_commands_P= NULL; /* pointer to the current line in the active sequence of commands, or NULL when none */
const int sensitive_pins[] = SENSITIVE_PINS; ///< Sensitive pin list for M42
// Inactivity shutdown
unsigned long previous_millis_cmd = 0;
static unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
unsigned long starttime = 0; ///< Print job start time
unsigned long stoptime = 0;  ///< Print job stop time
static uint8_t target_extruder;
bool Stopped = false;
bool CooldownNoWait = true;
bool target_direction;
static bool home_all_axis = true;



#define FW_V 1.0907

// Lifetime manage
unsigned long rpi_last_active = 0;
unsigned long rpi_wifi_active = 0;
bool rpi_io1_flag;
bool rpi_io2_flag;


//aven_0509
long setpoint=0;
int freq=0;
int duty=0;
int power=0;
long read_temp=0;
int pleds=0;
int qleds=0;
int f_led1=0;
int f_led2=0;
int f_led3=0;
int cled1=0;
int cled2=0;
int cled3=0;
int fled1=0;
int fled2=0;
int fled3=0;
int leds1=0;
int leds2=0;
int leds3=0;
int runleds1=0;
int runleds2=0;
int runleds3=0;
int dcount1=0;
int dcount2=0;
int dcount3=0;

int motors=0;
int motorc=0;

int filrunout_flag=0;
int filrunout_info_count=0;



float k_value= 0.03;
float u_value;
//char setpoint;
//char pbuf[5];

//aven_0721
int line_check = 0;
int NO = 0;

//aven_0807
int G28_f = 0;

//aven_0812
#define ROUND(x, y) (roundf(x * (float)(1e ## y)) / (float)(1e ## y))

float cartesian[3] = { 0, 0, 0 };
float odelta[3] = { 0, 0, 0 };
float cdelta[3] = { 0, 0, 0 };


//aven_0815
float h_offset=0;

#ifndef DELTA
  int xy_travel_speed = XY_TRAVEL_SPEED;
  float zprobe_zoffset = 0;
#endif

#if defined(Z_DUAL_ENDSTOPS) && !defined(DELTA)
  float z_endstop_adj = 0;
#endif

// Hotend offset
#if HOTENDS > 1
  #ifndef DUAL_X_CARRIAGE
    #define NUM_HOTEND_OFFSETS 2 // only in XY plane
  #else
    #define NUM_HOTEND_OFFSETS 3 // supports offsets in XYZ plane
  #endif
  float hotend_offset[NUM_HOTEND_OFFSETS][HOTENDS];
#endif

#ifdef NPR2
  int old_color = 99;
#endif

#if NUM_SERVOS > 0
  int servo_endstops[] = SERVO_ENDSTOPS;
  int servo_endstop_angles[] = SERVO_ENDSTOP_ANGLES;
#endif

#ifdef BARICUDA
  int ValvePressure = 0;
  int EtoPPressure = 0;
#endif

#ifdef FWRETRACT

  bool autoretract_enabled = false;
  bool retracted[EXTRUDERS] = { false };
  bool retracted_swap[EXTRUDERS] = { false };

  float retract_length = RETRACT_LENGTH;
  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_feedrate = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
  float retract_recover_feedrate = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

#if defined(ULTIPANEL) && HAS_POWER_SWITCH
  bool powersupply = 
    #ifdef PS_DEFAULT_OFF
      false
    #else
      true
    #endif
  ;
#endif

#ifdef DELTA

  float saved_endstop_adj[3] = { 0 }; //aven_0813
  float endstop_adj[3] = { 0, 0, 0 };
  float tower_adj[6] = { 0, 0, 0, 0, 0, 0 };
  float delta_radius; // = DEFAULT_delta_radius;
  float delta_diagonal_rod; // = DEFAULT_DELTA_DIAGONAL_ROD;
  float DELTA_DIAGONAL_ROD_2;
  float ac_prec = AUTOCALIBRATION_PRECISION / 2;
  float bed_radius = PRINTER_RADIUS;
  float delta_tower1_x, delta_tower1_y;
  float delta_tower2_x, delta_tower2_y;
  float delta_tower3_x, delta_tower3_y;
  float base_max_pos[3] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS};
  float base_home_pos[3] = {X_HOME_POS, Y_HOME_POS, Z_HOME_POS};
  float max_length[3] = {X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH};
  float saved_position[3] = { 0.0, 0.0, 0.0 };
  float saved_positions[7][3] = {
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      { 0, 0, 0 },
      };
  float delta[3] = { 0.0, 0.0, 0.0 };
  float delta_tmp[3] = { 0.0, 0.0, 0.0 };
  float probing_feedrate = PROBING_FEEDRATE;
  float default_z_probe_offset[] = Z_PROBE_OFFSET;
  float z_probe_offset[3];
  float z_probe_deploy_start_location[] = Z_PROBE_DEPLOY_START_LOCATION;
  float z_probe_deploy_end_location[] = Z_PROBE_DEPLOY_END_LOCATION;
  float z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION;
  float z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;
  #define SIN_60 0.8660254037844386
  #define COS_60 0.5
  static float bed_level[7][7] = {
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      { 0, 0, 0, 0, 0, 0, 0 },
      };

  static float adj_t1_Radius = 0;
  static float adj_t2_Radius = 0;
  static float adj_t3_Radius = 0;
  
  static float z_offset;
  static float bed_level_x, bed_level_y, bed_level_z;
  static float bed_safe_z = 45; //used for initial bed probe safe distance (to avoid crashing into bed) //aven_0813
  static float bed_level_c = 20; //used for inital bed probe safe distance (to avoid crashing into bed)
  static float bed_level_ox, bed_level_oy, bed_level_oz;
  static int loopcount;
#endif // DELTA

#ifdef SCARA
  static float delta[3] = { 0 };
  float axis_scaling[3] = { 1, 1, 1 };    // Build size scaling, default to 1
#endif

#if HAS_FILAMENT_SENSOR
  //Variables for Filament Sensor input 
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA;  //Set nominal filament width, can be changed with M404 
  bool filament_sensor = false;                                 //M405 turns on filament_sensor control, M406 turns it off 
  float filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;    //Stores the measured filament diameter 
  signed char measurement_delay[MAX_MEASUREMENT_DELAY+1];       //ring buffer to delay measurement  store extruder factor after subtracting 100 
  int delay_index1 = 0;                                         //index into ring buffer
  int delay_index2 = -1;                                        //index into ring buffer - set to -1 on startup to indicate ring buffer needs to be initialized
  float delay_dist = 0;                                         //delay distance counter
  int meas_delay_cm = MEASUREMENT_DELAY_CM;                     //distance delay setting
#endif

//#ifdef FILAMENT_RUNOUT_SENSOR //aven_test0826
  static bool filrunoutEnqued = false;
  bool printing = false;
//#endif

#ifdef SDSUPPORT
  static bool fromsd[BUFSIZE];
#endif //!SDSUPPORT

//#ifdef FILAMENTCHANGEENABLE //aven_test0826
	bool filament_changing = false;
//#endif

#if defined(IDLE_OOZING_PREVENT) || defined(EXTRUDER_RUNOUT_PREVENT)
  unsigned long axis_last_activity = 0;
  bool axis_is_moving = false;
#endif

#ifdef IDLE_OOZING_PREVENT
  bool idleoozing_enabled = true;
  bool IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false, false, false, false);
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  float power_consumption_meas = 0.0;
  unsigned long power_consumption_hour;
  unsigned long startpower = 0;
  unsigned long stoppower = 0;
#endif

#ifdef LASERBEAM
  int laser_ttl_modulation = 0;
#endif

#ifdef NPR2
  static float color_position[] = COLOR_STEP;  //variabile per la scelta del colore
  static float color_step_moltiplicator = (DRIVER_MICROSTEP / MOTOR_ANGLE) * CARTER_MOLTIPLICATOR;
#endif // NPR2

#ifdef EASY_LOAD
  bool allow_lengthy_extrude_once; // for load/unload
#endif

#if NUM_SERVOS > 0
  Servo servos[NUM_SERVOS];
#endif

#ifdef CHDK
  unsigned long chdkHigh = 0;
  boolean chdkActive = false;
#endif

//===========================================================================
//================================ Functions ================================
//===========================================================================
class Timer
{
  public:
    Timer(void);
    void set_max_delay(unsigned long v);
    void set(void);
    boolean check(void);
  private:
    unsigned long max_delay;
    unsigned long last_set;
};
Timer::Timer(void)
{
  max_delay = 3600000UL; // default 1 hour
}
void Timer::set_max_delay(unsigned long v)
{
  max_delay = v;
  set();
}
void Timer::set()
{
  last_set = millis();
}
boolean Timer::check()
{
  unsigned long now = millis();
  if (now - last_set > max_delay) {
    last_set = now;
    return true;
  }
  return false;
}
Timer timer;

void get_arc_coordinates();
bool setTargetedHotend(int code);

void serial_echopair_P(const char *s_P, float v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, double v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char *s_P, unsigned long v)
    { serialprintPGM(s_P); SERIAL_ECHO(v); }

#ifndef __SAM3X8E__
#ifdef SDSUPPORT
  #include "SdFatUtil.h"
  int freeMemory() { return SdFatUtil::FreeRam(); }
#else
  extern "C" {
    extern unsigned int __bss_end;
    extern unsigned int __heap_start;
    extern void *__brkval;

    int freeMemory() {
      int free_memory;

      if ((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
      else
        free_memory = ((int)&free_memory) - ((int)__brkval);

      return free_memory;
    }
  }
#endif //!SDSUPPORT
#endif

//Injects the next command from the pending sequence of commands, when possible
//Return false if and only if no command was pending
static bool drain_queued_commands_P() {
  if (!queued_commands_P) return false;

  // Get the next 30 chars from the sequence of gcodes to run
  char cmd[30];
  strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Look for the end of line, or the end of sequence
  size_t i = 0;
  char c;
  while((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
  cmd[i] = '\0';
  if (enquecommand(cmd)) {        // buffer was not full (else we will retry later)
    if (c)
      queued_commands_P += i + 1; // move to next command
    else
      queued_commands_P = NULL;   // will have no more commands in the sequence
  }

  return true;
}

//Record one or many commands to run from program memory.
//Aborts the current queue, if any.
//Note: drain_queued_commands_P() must be called repeatedly to drain the commands afterwards
void enquecommands_P(const char* pgcode) {
    queued_commands_P = pgcode;
    drain_queued_commands_P(); // first command executed asap (when possible)
}

//adds a single command to the main command buffer, from RAM
//that is really done in a non-safe way.
//needs overworking someday
//Returns false if it failed to do so
bool enquecommand(const char *cmd)
{
  if(*cmd==';')
    return false;
  if(buflen >= BUFSIZE)
    return false;
  //this is dangerous if a mixing of serial and this happens
  strcpy(&(cmdbuffer[bufindw][0]),cmd);
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_Enqueing);
  SERIAL_ECHO(cmdbuffer[bufindw]);
  SERIAL_ECHOLNPGM("\"");
  bufindw= (bufindw + 1)%BUFSIZE;
  buflen += 1;
  return true;
}


#if MB(ALLIGATOR)
  void setup_alligator_board() {
    // Init Expansion Port Voltage logic Selector
    SET_OUTPUT(EXP_VOLTAGE_LEVEL_PIN);
    WRITE(EXP_VOLTAGE_LEVEL_PIN,UI_VOLTAGE_LEVEL);
    ExternalDac::begin(); //initialize ExternalDac
  }
#endif

void setup_killpin()
{
  #if HAS_KILL
    SET_INPUT(KILL_PIN);
    WRITE(KILL_PIN, HIGH);
  #endif
}

void setup_filrunoutpin()
{
  #if HAS_FILRUNOUT 
    pinMode(FILRUNOUT_PIN, INPUT);
    #ifdef ENDSTOPPULLUP_FIL_RUNOUT
      WRITE(FILLRUNOUT_PIN, HIGH);
   #endif
  #endif
}

// Set home pin
void setup_homepin(void)
{
  #if HAS_HOME
    SET_INPUT(HOME_PIN);
    WRITE(HOME_PIN, HIGH);
  #endif
}


void setup_photpin()
{
  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

void setup_laserbeampin()
{
  #ifdef LASERBEAM
    OUT_WRITE(LASER_PWR_PIN, LOW);
    OUT_WRITE(LASER_TTL_PIN, LOW);
  #endif
}

void setup_powerhold()
{
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #ifdef PS_DEFAULT_OFF
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}

void suicide()
{
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init()
{
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servos[0].attach(SERVO0_PIN);
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servos[1].attach(SERVO1_PIN);
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servos[2].attach(SERVO2_PIN);
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servos[3].attach(SERVO3_PIN);
  #endif

  // Set position of Servo Endstops that are defined
  #if (NUM_SERVOS > 0)
    for(int i = 0; i < 3; i++)
      if(servo_endstops[i] > -1)
        servos[servo_endstops[i]].write(servo_endstop_angles[i * 2 + 1]);
  #endif //NUM_SERVOS

  #if SERVO_LEVELING
    delay(PROBE_SERVO_DEACTIVATION_DELAY);
    servos[servo_endstops[Z_AXIS]].detach();
  #endif
}

// manage_led
#define _led_wave_atom(a, b) pow(sin(a * ((float)millis() - b)), 2)
#define _led_wave(i) _led_wave_atom(led_param_a[i], led_param_b[i])
#define LED_OFF 0
#define LED_WAVE 1
#define LED_BLINK 2
#define LED_ON 3
#define LED_WAVE_2_ON 4
#define LED_WAVE_2_OFF 5

int led_pins[3] = {LED_P1, LED_P2, LED_P3};
char led_mode = 'W';  // 'W'akeup, 'R'eady, 'S'leep, 'F'atel
float led_param_a[3] = {0.0009, 1, 1};
float led_param_b[3] = {0, 0, 0};
char led_ctrl[3] = {1, 0, 0};

// pre-molloc space
char new_led_mode;

void manage_led()
{
  new_led_mode = led_mode;

  if(rpi_io1_flag == (digitalRead(R_IO1) == HIGH)) {
    // rpi does not change R_IO1 status, check if timeout
    if(rpi_last_active) {
      // TODO: 5000
      if(millis() - rpi_last_active > 500000) {
        new_led_mode = 'F';
      } else {
        new_led_mode = 'R';
      }
    }
  } else {
    // rpi change R_IO1 flag, update status
    rpi_io1_flag = !rpi_io1_flag;
    rpi_last_active = millis();
    new_led_mode = 'R';
  }

  if(new_led_mode == 'R') {
    if(rpi_io2_flag == (digitalRead(R_IO2) == HIGH)) {
      if(millis() - rpi_wifi_active > 5000) {
        // rpi wifi status does not change over 5s
        if(rpi_io2_flag && led_ctrl[2] != LED_ON)  // wifi is up
          led_ctrl[2] = LED_WAVE_2_ON;
        else if((!rpi_io2_flag) && led_ctrl[2] != LED_OFF)  // sleep
          new_led_mode = 'S';
      }
    } else {
      // rpi wifi status is changed, wave led
      rpi_io2_flag = !rpi_io2_flag;
      rpi_wifi_active = millis();
      if(led_ctrl[2] != LED_WAVE) {
        led_ctrl[2] = LED_WAVE;
        led_param_a[2] = 0.0009;
        led_param_b[2] = millis();
      }
    }
  }

  if(new_led_mode != led_mode) {
    led_mode = new_led_mode;
    switch(led_mode) {
      case 'R':
        led_ctrl[0] = LED_WAVE_2_ON;
        led_ctrl[1] = LED_OFF;
        break;
      case 'F':
        led_ctrl[0] = LED_BLINK; led_param_a[0] = 0.003; led_param_b[0] = 0;
        led_ctrl[1] = LED_BLINK; led_param_a[1] = 0.003; led_param_b[1] = 0;
        led_ctrl[2] = LED_OFF;
        break;
      case 'S':
        led_ctrl[0] = led_ctrl[1] = led_ctrl[2] = 0;
        break;
      case 'W':
        break;
      default:
        led_ctrl[0] = led_ctrl[1] = led_ctrl[2] = LED_WAVE;
        led_param_a[0] = led_param_a[1] = led_param_a[2] = 0.005;
        led_param_b[0] = led_param_b[2] = 0;
        led_param_b[1] = 175;
    }
  }

  for(int i=0;i<3;i++) {
    switch(led_ctrl[i]) {
      case LED_OFF:
        analogWrite(led_pins[i], 0);
        break;
      case LED_WAVE:
        analogWrite(led_pins[i], int(_led_wave(i) * 255));
        break;
      case LED_BLINK:
        analogWrite(led_pins[i], (_led_wave(i) > 0.5) ? 255 : 0);
        break;
      case LED_ON:
        analogWrite(led_pins[i], 255);
        break;
      case LED_WAVE_2_ON:
        if(_led_wave(i) > 0.95) {
          analogWrite(led_pins[i], 255);
          led_ctrl[i] = LED_ON;
        }
        else {
          analogWrite(led_pins[i], int(_led_wave(i) * 255));
        }
        break;
      case LED_WAVE_2_OFF:
      if(_led_wave(i) < 0.05) {
        analogWrite(led_pins[i], 0);
        led_ctrl[i] = LED_OFF;
      }
      else {
        analogWrite(led_pins[i], int(_led_wave(i) * 255));
      }
      break;
        break;
      default:
        analogWrite(led_pins[i], 0);
        break;
    }
  }
}


void setup()
{

//aven_0509
  Serial.begin(115200);
  SerialUSB.begin(115200);

  #if MB(ALLIGATOR)
    setup_alligator_board();// Initialize Alligator Board
  #endif
  setup_killpin();
  //setup_filrunoutpin(); //aven_test0826
  setup_powerhold();
  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START;

  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if(mcu & 1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if(mcu & 2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if(mcu & 4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if(mcu & 8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if(mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR=0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_ECHOLNPGM(STRING_VERSION);
  #ifdef STRING_VERSION_CONFIG_H
    #ifdef STRING_CONFIG_H_AUTHOR
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_CONFIGURATION_VER);
      SERIAL_ECHOPGM(STRING_VERSION_CONFIG_H);
      SERIAL_ECHOPGM(MSG_AUTHOR);
      SERIAL_ECHOLNPGM(STRING_CONFIG_H_AUTHOR);
      SERIAL_ECHOPGM("Compiled: ");
      SERIAL_ECHOLNPGM(__DATE__);
    #endif // STRING_CONFIG_H_AUTHOR
  #endif // STRING_VERSION_CONFIG_H
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_FREE_MEMORY);
  SERIAL_ECHO(freeMemory());
  SERIAL_ECHOPGM(MSG_PLANNER_BUFFER_BYTES);
  SERIAL_ECHOLN((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  Config_RetrieveSettings();

  tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!
  setup_photpin();
  setup_laserbeampin();   // Initialize Laserbeam pin
  servo_init();
  
  lcd_init();
  _delay_ms(1000);  // wait 1sec to display the splash screen

  #if HAS_CONTROLLERFAN
    SET_OUTPUT(CONTROLLERFAN_PIN); //Set pin used for driver cooling fan
  #endif

  #ifdef DIGIPOT_I2C
    digipot_i2c_init();
  #endif

  #ifdef Z_PROBE_SLED
    OUT_WRITE(SERVO0_PIN, LOW); // turn it off
  #endif // Z_PROBE_SLED

  setup_homepin();

  #ifdef STAT_LED_RED
    pinMode(STAT_LED_RED, OUTPUT);
    digitalWrite(STAT_LED_RED, LOW); // turn it off
  #endif
  #ifdef STAT_LED_BLUE
    pinMode(STAT_LED_BLUE, OUTPUT);
    digitalWrite(STAT_LED_BLUE, LOW); // turn it off
  #endif

  #ifdef FIRMWARE_TEST
    FirmwareTest();
  #endif // FIRMWARE_TEST

//aven_0415_2015 add cmd for S_LSA1 & S_LSA2 on/off start
  pinMode(S_LAS1, OUTPUT);//PC25
  pinMode(S_LAS2, OUTPUT);//PC26

//aven 0504 - TI TPS2552
  pinMode(U5EN,OUTPUT);
  pinMode(U5FAULT,INPUT);

  //digitalWrite(U5EN, HIGH);
  digitalWrite(U5EN, LOW);

//aven 0708
  pinMode(M_IO1,INPUT);//PD7

//aven_0825
  pinMode(M_IO2,OUTPUT);//PD8


  pinMode(CAP_IO,INPUT);
  pinMode(REF_IO,INPUT);

  // Initial LED
  pinMode(LED_P1,OUTPUT);
  pinMode(LED_P2,OUTPUT);
  pinMode(LED_P3,OUTPUT);

  digitalWrite(S_LAS1, LOW);
  digitalWrite(S_LAS2, LOW);
  
  analogWrite(LED_P1, 0);
  analogWrite(LED_P2, 0);
  analogWrite(LED_P3, 0); 
   

//aven 0504 - HOME_K  
  pinMode(HOME_K,INPUT);

  //fan_run();

  // LED Control
  pinMode(R_IO1,INPUT);
  pinMode(R_IO2,INPUT);
  rpi_io1_flag = digitalRead(R_IO1) == HIGH;
  rpi_io2_flag = digitalRead(R_IO1) == HIGH;
  led_param_b[0] = millis();


  //aven 0716
  pinMode(REFC1,OUTPUT); 
  pinMode(REFC2,OUTPUT);
  pinMode(REFC3,OUTPUT); 
  pinMode(REFC4,OUTPUT);

  digitalWrite(REFC1, LOW);
  digitalWrite(REFC2, LOW);
  digitalWrite(REFC3, LOW);
  digitalWrite(REFC4, LOW);

  //aven 0724
  pinMode(EN1,OUTPUT); 
  pinMode(EN2,OUTPUT);
  pinMode(EN3,OUTPUT); 
  pinMode(EN4,OUTPUT);
  pinMode(EN5,OUTPUT);
  pinMode(EN6,OUTPUT);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(EN3, HIGH);
  digitalWrite(EN4, HIGH);
  digitalWrite(EN5, HIGH);
  digitalWrite(EN6, HIGH);

  pinMode(F0_STOP,INPUT);
  digitalWrite(F0_STOP, HIGH);


}

void loop() {
  
  if (buflen < BUFSIZE - 1) get_command();

  #ifdef SDSUPPORT
    card.checkautostart(false);
  #endif

  if (buflen) {
    #ifdef SDSUPPORT
      if (card.saving) {
        if (strstr_P(cmdbuffer[bufindr], PSTR("M29")) == NULL) {
          card.write_command(cmdbuffer[bufindr]);
          if (card.logging)
            process_commands();
          else
            SERIAL_PROTOCOLLNPGM(MSG_OK);
        }
        else {
          card.closefile();
          SERIAL_PROTOCOLLNPGM(MSG_FILE_SAVED);
        }
      }
      else
        process_commands();
    #else
      process_commands();
    #endif // SDSUPPORT
    buflen--;
    bufindr = (bufindr + 1) % BUFSIZE;
  }
  // Check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  //lcd_update();
  manage_led();

  //SerialUSB.println(digitalRead(F0_STOP));
  //SerialUSB.println(digitalRead(FILRUNOUT_PIN));
  
}


bool inline check_line_number(const char* cmd) {
  // Check N
  strchr_pointer = strchr(cmd, 'N');

  if(strchr_pointer == NULL) {
    // Can not found Line Number, send error, clean buffer and return
    SERIAL_PROTOCOL("ER MISSING_LINENUMBER ");
    SERIAL_PROTOCOL(gcode_LastN + 1);
    SERIAL_PROTOCOL("\n");
    MYSERIAL.flush();
    return false;

  } else {
    gcode_N = (strtol(strchr_pointer + 1, NULL, 10));
    if(gcode_N != gcode_LastN + 1) {
      SERIAL_PROTOCOL("ER LINE_MISMATCH ");
      SERIAL_PROTOCOL(gcode_LastN + 1);
      SERIAL_PROTOCOL(" ");
      SERIAL_PROTOCOL(gcode_N);
      SERIAL_PROTOCOL("\n");
      MYSERIAL.flush();
      return false;
    }
  }

  // Check checksum
  strchr_pointer = strchr(cmd, '*');
  if(strchr_pointer == NULL) {
    // Checksum not send
    SERIAL_PROTOCOL("ER CHECKSUM_MISMATCH ");
    SERIAL_PROTOCOL(gcode_LastN + 1);
    SERIAL_PROTOCOL("\n");
    MYSERIAL.flush();
    return false;

  } else {
    // Calculate checksum
    byte checksum = 0;
    byte count = 0;

    while(cmd[count] != '*') {
      checksum = checksum^cmd[count++];
    }

    strchr_pointer = strchr(cmd, '*');

    if(strtol(strchr_pointer + 1, NULL, 10) != checksum) {
      // Checksum not match
      SERIAL_PROTOCOL("ER CHECKSUM_MISMATCH ");
      SERIAL_PROTOCOL(gcode_LastN + 1);
      SERIAL_PROTOCOL("\n");
      MYSERIAL.flush();
      return false;
    }
  }

  // No error, update last line code
  SERIAL_PROTOCOL("LN ");
  SERIAL_PROTOCOL(gcode_N);
  SERIAL_PROTOCOL(" ");
  SERIAL_PROTOCOL(buflen);
  SERIAL_PROTOCOL("\n");
  MYSERIAL.flush();

  gcode_LastN = gcode_N;
  return true;
}


void get_command()
{
  if (drain_queued_commands_P()) // priority is given to non-serial commands
    return;

  while( MYSERIAL.available() > 0  && buflen < BUFSIZE) {
    serial_char = MYSERIAL.read();
    if(serial_char == '\n' ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) {
        // short cut for empty lines
        return;
      }

      cmdbuffer[bufindw][serial_count] = 0; //terminate string

      //If command was e-stop process now
      if(strcmp(cmdbuffer[bufindw], "M112") == 0)
        kill();

      if(line_check == 1) {
        if(!check_line_number(cmdbuffer[bufindw])) {
          // Check failed, ignore this command and return
          serial_count = 0;
          return;
        }
      } else {
        if(strchr(cmdbuffer[bufindw], 'N') != NULL) {
          SERIAL_PROTOCOLLN("ER NCODE_NOT_ACCEPTED");
          serial_count = 0;
          return;
        }
      }

      if((strchr(cmdbuffer[bufindw], 'G') != NULL)) {
        strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
        switch(strtol(strchr_pointer + 1, NULL, 10)) {
        case 0:
        case 1:
        case 2:
        case 3:
          if (Stopped == true) {
            SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
            LCD_MESSAGEPGM(MSG_STOPPED);
          }
          break;
        default:
          break;
        }
      }

      bufindw = (bufindw + 1)%BUFSIZE;
      buflen += 1;

      serial_count = 0; //clear buffer
    }
    else {   // its not a newline, carriage return or escape char
      cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
}

float code_value() {
  float ret;
  char *e = strchr(strchr_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(strchr_pointer+1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(strchr_pointer+1, NULL);
  return ret;
}

long code_value_long() { return strtol(strchr_pointer + 1, NULL, 10); }

int16_t code_value_short() { return (int16_t)strtol(strchr_pointer + 1, NULL, 10); }

//aven test
float code_value_float() { return strtol(strchr_pointer + 1, NULL, 10); }
//aven test

bool code_seen(char code) {
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

#define DEFINE_PGM_READ_ANY(type, reader)       \
    static inline type pgm_read_any(const type *p)  \
    { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float);
DEFINE_PGM_READ_ANY(signed char, byte);

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
static const PROGMEM type array##_P[3] =        \
    { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
static inline type array(int axis)          \
    { return pgm_read_any(&array##_P[axis]); }

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS);
#if defined(CARTESIAN) || defined(COREXY) || defined(SCARA)
  XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,    MAX_POS);
  XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,   HOME_POS);
  XYZ_CONSTS_FROM_CONFIG(float, max_length,      MAX_LENGTH);
#endif
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm, HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR);

#ifdef DUAL_X_CARRIAGE

  #define DXC_FULL_CONTROL_MODE 0
  #define DXC_AUTO_PARK_MODE    1
  #define DXC_DUPLICATION_MODE  2

  static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

  static float x_home_pos(int extruder) {
    if (extruder == 0)
      return base_home_pos(X_AXIS) + home_offset[X_AXIS];
    else
      // In dual carriage mode the extruder offset provides an override of the
      // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
      // This allow soft recalibration of the second extruder offset position without firmware reflash
      // (through the M218 command).
      return (hotend_offset[X_AXIS][1] > 0) ? hotend_offset[X_AXIS][1] : X2_HOME_POS;
  }

  static int x_home_dir(int extruder) {
    return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
  }

  static float inactive_extruder_x_pos = X2_MAX_POS; // used in mode 0 & 1
  static bool active_extruder_parked = false; // used in mode 1 & 2
  static float raised_parked_position[NUM_AXIS]; // used in mode 1
  static unsigned long delayed_move_time = 0; // used in mode 1
  static float duplicate_extruder_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
  static float duplicate_extruder_temp_offset = 0; // used in mode 2
  bool extruder_duplication_enabled = false; // used in mode 2

#endif //DUAL_X_CARRIAGE

// Some planner shorthand inline functions
inline void line_to_current_position() {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder, active_driver);
}
inline void line_to_z(float zPosition) {
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder, active_driver);
}
inline void line_to_destination() {
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder, active_driver);
}
inline void sync_plan_position() {
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#if defined(DELTA) || defined(SCARA)
  inline void sync_plan_position_delta() {
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
  }
#endif
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

#if 0 //aven_0813
#if defined(CARTESIAN) || defined(COREXY) || defined(SCARA)
  static void axis_is_at_home(int axis) {

    #ifdef DUAL_X_CARRIAGE
      if (axis == X_AXIS) {
        if (active_extruder != 0) {
          current_position[X_AXIS] = x_home_pos(active_extruder);
                   min_pos[X_AXIS] = X2_MIN_POS;
                   max_pos[X_AXIS] = max(hotend_offset[X_AXIS][1], X2_MAX_POS);
          return;
        }
        else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
          float xoff = home_offset[X_AXIS];
          current_position[X_AXIS] = base_home_pos(X_AXIS) + xoff;
                   min_pos[X_AXIS] = base_min_pos(X_AXIS) + xoff;
                   max_pos[X_AXIS] = min(base_max_pos(X_AXIS) + xoff, max(hotend_offset[X_AXIS][1], X2_MAX_POS) - duplicate_extruder_x_offset);
          return;
        }
      }
    #endif

    #ifdef SCARA
      float homeposition[3];

      if (axis < 2) {
        for (int i = 0; i < 3; i++) homeposition[i] = base_home_pos(i);

        // SERIAL_ECHOPGM("homeposition[x]= "); SERIAL_ECHO(homeposition[0]);
        // SERIAL_ECHOPGM("homeposition[y]= "); SERIAL_ECHOLN(homeposition[1]);
        // Works out real Homeposition angles using inverse kinematics, 
        // and calculates homing offset using forward kinematics
        calculate_delta(homeposition);

        // SERIAL_ECHOPGM("base Theta= "); SERIAL_ECHO(delta[X_AXIS]);
        // SERIAL_ECHOPGM(" base Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

        for (int i = 0; i < 2; i++) delta[i] -= home_offset[i];

        // SERIAL_ECHOPGM("addhome X="); SERIAL_ECHO(home_offset[X_AXIS]);
        // SERIAL_ECHOPGM(" addhome Y="); SERIAL_ECHO(home_offset[Y_AXIS]);
        // SERIAL_ECHOPGM(" addhome Theta="); SERIAL_ECHO(delta[X_AXIS]);
        // SERIAL_ECHOPGM(" addhome Psi+Theta="); SERIAL_ECHOLN(delta[Y_AXIS]);

        calculate_SCARA_forward_Transform(delta);

        // SERIAL_ECHOPGM("Delta X="); SERIAL_ECHO(delta[X_AXIS]);
        // SERIAL_ECHOPGM(" Delta Y="); SERIAL_ECHOLN(delta[Y_AXIS]);

        current_position[axis] = delta[axis];

        // SCARA home positions are based on configuration since the actual limits are determined by the 
        // inverse kinematic transform.
        min_pos[axis] = base_min_pos(axis); // + (delta[axis] - base_home_pos(axis));
        max_pos[axis] = base_max_pos(axis); // + (delta[axis] - base_home_pos(axis));
      } 
      else {
        current_position[axis] = base_home_pos(axis) + home_offset[axis];
        min_pos[axis] = base_min_pos(axis) + home_offset[axis];
        max_pos[axis] = base_max_pos(axis) + home_offset[axis];
      }
    #else
      current_position[axis] = base_home_pos(axis) + home_offset[axis];
      min_pos[axis] = base_min_pos(axis) + home_offset[axis];
      max_pos[axis] = base_max_pos(axis) + home_offset[axis];
    #endif
  }

  static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = feedrate;
    feedrate = homing_feedrate[Z_AXIS];

    current_position[Z_AXIS] = z;
    line_to_current_position();
    st_synchronize();

    feedrate = xy_travel_speed;

    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    line_to_current_position();
    st_synchronize();

    feedrate = oldFeedRate;
  }
    
  #ifdef ENABLE_AUTO_BED_LEVELING
    #ifdef AUTO_BED_LEVELING_GRID
      static void set_bed_level_equation_lsq(double *plane_equation_coefficients) {
        vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
        planeNormal.debug("planeNormal");
        plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
        //bedLevel.debug("bedLevel");

        //plan_bed_level_matrix.debug("bed level before");
        //vector_3 uncorrected_position = plan_get_position_mm();
        //uncorrected_position.debug("position before");

        vector_3 corrected_position = plan_get_position();
        //corrected_position.debug("position after");
        current_position[X_AXIS] = corrected_position.x;
        current_position[Y_AXIS] = corrected_position.y;
        current_position[Z_AXIS] = corrected_position.z;

        sync_plan_position();
      }
    #else // not AUTO_BED_LEVELING_GRID
      static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

        plan_bed_level_matrix.set_to_identity();

        vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
        vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
        vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);
        vector_3 planeNormal = vector_3::cross(pt1 - pt2, pt3 - pt2).get_normal();

        if (planeNormal.z < 0) {
          planeNormal.x = -planeNormal.x;
          planeNormal.y = -planeNormal.y;
          planeNormal.z = -planeNormal.z;
        }

        plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

        vector_3 corrected_position = plan_get_position();
        current_position[X_AXIS] = corrected_position.x;
        current_position[Y_AXIS] = corrected_position.y;
        current_position[Z_AXIS] = corrected_position.z;

        sync_plan_position();
      }

    #endif // AUTO_BED_LEVELING_GRID

    static void run_z_probe() {

      plan_bed_level_matrix.set_to_identity();
      feedrate = homing_feedrate[Z_AXIS];

      // move down until you find the bed
      float zPosition = -10;
      line_to_z(zPosition);
      st_synchronize();

      // we have to let the planner know where we are right now as it is not where we said to go.
      zPosition = st_get_position_mm(Z_AXIS);
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);

      // move up the retract distance
      zPosition += home_bump_mm(Z_AXIS);
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags

      // move back down slowly to find bed
      if (homing_bump_divisor[Z_AXIS] >= 1) {
        feedrate = homing_feedrate[Z_AXIS] / homing_bump_divisor[Z_AXIS];
      }
      else {
        feedrate = homing_feedrate[Z_AXIS] / 10;
        SERIAL_ECHOLN("Warning: The Homing Bump Feedrate Divisor cannot be less than 1");
      }

      zPosition -= home_bump_mm(Z_AXIS) * 2;
      line_to_z(zPosition);
      st_synchronize();
      endstops_hit_on_purpose(); // clear endstop hit flags

      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
      // make sure the planner knows where we are as it may be a bit different than we last said to move to
      sync_plan_position();
    }

    static void do_blocking_move_relative(float offset_x, float offset_y, float offset_z) {
      do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
    }

    static void setup_for_endstop_move() {
      saved_feedrate = feedrate;
      saved_feedmultiply = feedmultiply;
      feedmultiply = 100;
      refresh_cmd_timeout();
      enable_endstops(true);
    }

    static void clean_up_after_endstop_move() {
      #ifdef ENDSTOPS_ONLY_FOR_HOMING
        enable_endstops(false);
      #endif
      feedrate = saved_feedrate;
      feedmultiply = saved_feedmultiply;
      refresh_cmd_timeout();
    }

    static void deploy_z_probe() {
      #if NUM_SERVOS > 0
        // Engage Z Servo endstop if enabled
        if (servo_endstops[Z_AXIS] >= 0) {

          #if SERVO_LEVELING
            servos[servo_endstops[Z_AXIS]].attach(0);
          #endif

          servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2]);

          #if SERVO_LEVELING
            delay(PROBE_SERVO_DEACTIVATION_DELAY);
            servos[servo_endstops[Z_AXIS]].detach();
          #endif
        }
      #endif //NUM_SERVOS > 0
    }

    static void stow_z_probe() {
      #if NUM_SERVOS > 0
        // Retract Z Servo endstop if enabled
        if (servo_endstops[Z_AXIS] >= 0) {

          /* NON FUNZIONA DA VERIFICARE
          #if Z_RAISE_AFTER_PROBING > 0
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_RAISE_AFTER_PROBING);
          #endif
          */

          #if SERVO_LEVELING
            servos[servo_endstops[Z_AXIS]].attach(0);
          #endif

          servos[servo_endstops[Z_AXIS]].write(servo_endstop_angles[Z_AXIS * 2 + 1]);

          #if SERVO_LEVELING
            delay(PROBE_SERVO_DEACTIVATION_DELAY);
            servos[servo_endstops[Z_AXIS]].detach();
          #endif
        }
      #endif //NUM_SERVOS > 0
    }

    enum ProbeAction {
      ProbeStay             = 0,
      ProbeDeploy           = BIT(0),
      ProbeStow             = BIT(1),
      ProbeDeployAndStow    = (ProbeDeployAndStow | ProbeStow)
    };

    // Probe bed height at position (x,y), returns the measured z value
    static float probe_pt(float x, float y, float z_before, ProbeAction retract_action=ProbeDeployAndStow, int verbose_level=1) {
      // move to right place
      do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before);
      do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, current_position[Z_AXIS]);

      #ifndef Z_PROBE_SLED
        if (retract_action & ProbeDeploy) deploy_z_probe();
      #endif // Z_PROBE_SLED

      run_z_probe();
      float measured_z = current_position[Z_AXIS];

      #if Z_RAISE_BETWEEN_PROBINGS > 0
        if (retract_action == ProbeStay) {
          do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);
          st_synchronize();
        }
      #endif

      #ifndef Z_PROBE_SLED
        if (retract_action & ProbeStow) stow_z_probe();
      #endif

      if (verbose_level > 2) {
        SERIAL_PROTOCOLPGM(MSG_BED);
        SERIAL_PROTOCOLPGM(" X: ");
        SERIAL_PROTOCOL_F(x, 3);
        SERIAL_PROTOCOLPGM(" Y: ");
        SERIAL_PROTOCOL_F(y, 3);
        SERIAL_PROTOCOLPGM(" Z: ");
        SERIAL_PROTOCOL_F(measured_z, 3);
        SERIAL_EOL;
      }
      return measured_z;
    }
  #endif //ENABLE_AUTO_BED_LEVELING

  static void homeaxis(int axis) {
  #define HOMEAXIS_DO(LETTER) \
    ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

    if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

      int axis_home_dir;

      #ifdef DUAL_X_CARRIAGE
        if (axis == X_AXIS) axis_home_dir = x_home_dir(active_extruder);
      #else
        axis_home_dir = home_dir(axis);
      #endif

      // Set the axis position as setup for the move
      current_position[axis] = 0;
      sync_plan_position();

      // Engage Servo endstop if enabled
      #if (NUM_SERVOS > 0) && !defined(Z_PROBE_SLED)
        #if SERVO_LEVELING
          if (axis == Z_AXIS) deploy_z_probe(); else
        #endif
          {
            if (servo_endstops[axis] > -1)
              servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2]);
          }
      #endif // SERVO_ENDSTOPS && !Z_PROBE_SLED

      #ifdef Z_DUAL_ENDSTOPS
        if (axis == Z_AXIS) In_Homing_Process(true);
      #endif

      // Move towards the endstop until an endstop is triggered
      destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
      feedrate = homing_feedrate[axis];
      line_to_destination();
      st_synchronize();

      // Set the axis position as setup for the move
      current_position[axis] = 0;
      sync_plan_position();

      // Move away from the endstop by the axis HOME_BUMP_MM
      destination[axis] = -home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      // Slow down the feedrate for the next move
      if (homing_bump_divisor[axis] >= 1) {
        feedrate = homing_feedrate[axis] / homing_bump_divisor[axis];
      }
      else {
        feedrate = homing_feedrate[axis] / 10;
        SERIAL_ECHOLN("Warning: The Homing Bump Feedrate Divisor cannot be less then 1");
      }

      // Move slowly towards the endstop until triggered
      destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      #ifdef Z_DUAL_ENDSTOPS
        if (axis == Z_AXIS) {
          float adj = fabs(z_endstop_adj);
          bool lockZ1;
          if (axis_home_dir > 0) {
            adj = -adj;
            lockZ1 = (z_endstop_adj > 0);
          }
          else
            lockZ1 = (z_endstop_adj < 0);

          if (lockZ1) Lock_z_motor(true); else Lock_z2_motor(true);
          sync_plan_position();

          // Move to the adjusted endstop height
          feedrate = homing_feedrate[axis];
          destination[Z_AXIS] = adj;
          line_to_destination();
          st_synchronize();

          if (lockZ1) Lock_z_motor(false); else Lock_z2_motor(false);
          In_Homing_Process(false);
        } // Z_AXIS
      #endif

      // Set the axis position to its home position (plus home offsets)
      axis_is_at_home(axis);

      destination[axis] = current_position[axis];
      feedrate = 0.0;
      endstops_hit_on_purpose(); // clear endstop hit flags
      axis_known_position[axis] = true;

      // Retract Servo endstop if enabled
      #if NUM_SERVOS > 0
        if (servo_endstops[axis] >= 0)
          servos[servo_endstops[axis]].write(servo_endstop_angles[axis * 2 + 1]);
      #endif

      #if SERVO_LEVELING && !defined(Z_PROBE_SLED)
        if (axis == Z_AXIS) stow_z_probe();
      #endif
    }
  }
  #define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)
#endif // Cartesian || CoreXY || Scara

#endif //aven_0813


#ifdef DELTA
  static void axis_is_at_home(int axis) {
    current_position[axis] = base_home_pos[axis] + home_offset[axis];
    min_pos[axis] =          base_min_pos(axis) + home_offset[axis];
    max_pos[axis] =          base_max_pos[axis] + home_offset[axis];
  }

  static void homeaxis(int axis)
  {
    #define HOMEAXIS_DO(LETTER) \
      ((LETTER##_MIN_PIN > -1 && LETTER##_HOME_DIR==-1) || (LETTER##_MAX_PIN > -1 && LETTER##_HOME_DIR==1))

    if (axis == X_AXIS ? HOMEAXIS_DO(X) : axis == Y_AXIS ? HOMEAXIS_DO(Y) : axis == Z_AXIS ? HOMEAXIS_DO(Z) : 0) {

      int axis_home_dir = home_dir(axis);
      current_position[axis] = 0;
      sync_plan_position();
      destination[axis] = 1.5 * max_length[axis] * axis_home_dir;
      feedrate = homing_feedrate[axis];
      line_to_destination();
      st_synchronize();

      enable_endstops(false);  // Ignore Z probe while moving away from the top microswitch.
      current_position[axis] = 0;
      sync_plan_position();
      destination[axis] = -home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();
      enable_endstops(true);  // Stop ignoring Z probe while moving up to the top microswitch again.

      // Slow down the feedrate for the next move
      if (homing_bump_divisor[axis] >= 1)
        feedrate = homing_feedrate[axis] / homing_bump_divisor[axis];
      else {
        feedrate = homing_feedrate[axis] / 10;
        SERIAL_ECHOLNPGM("Warning: The Homing Bump Feedrate Divisor cannot be less than 1");
      }

      // Move slowly towards the endstop until triggered
      destination[axis] = 2 * home_bump_mm(axis) * axis_home_dir;
      line_to_destination();
      st_synchronize();

      // retrace by the amount specified in endstop_adj
      if (endstop_adj[axis] * axis_home_dir < 0)
      {
        enable_endstops(false);  // Ignore Z probe while moving away from the top microswitch.
        sync_plan_position();
        destination[axis] = endstop_adj[axis];
        line_to_destination();
        st_synchronize();
        enable_endstops(true);  // Stop ignoring Z probe while moving up to the top microswitch again.
      }

      // Set the axis position to its home position (plus home offsets)
      axis_is_at_home(axis);

      destination[axis] = current_position[axis];
      feedrate = 0.0;
      endstops_hit_on_purpose(); // clear endstop hit flags
      axis_known_position[axis] = true;
    }
  }
  #define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

  void set_default_z_probe_offset()
  {
    z_probe_offset[X_AXIS] = default_z_probe_offset[X_AXIS];
    z_probe_offset[Y_AXIS] = default_z_probe_offset[Y_AXIS];
    z_probe_offset[Z_AXIS] = default_z_probe_offset[Z_AXIS];
  }

  void set_delta_constants()
  {
    max_length[Z_AXIS] = max_pos[Z_AXIS] - Z_MIN_POS;
    base_max_pos[Z_AXIS]  = max_pos[Z_AXIS];
    base_home_pos[Z_AXIS] = max_pos[Z_AXIS];

    DELTA_DIAGONAL_ROD_2 = pow(delta_diagonal_rod,2);

    // Effective X/Y positions of the three vertical towers.
    /*
    delta_tower1_x = (-SIN_60 * delta_radius) + tower_adj[0]; // front left tower + xa
    delta_tower1_y = (-COS_60 * delta_radius) - tower_adj[0] ;
    delta_tower2_x = -(-SIN_60 * delta_radius) + tower_adj[1]; // front right tower + xb
    delta_tower2_y = (-COS_60 * delta_radius) + tower_adj[1]; // 
    delta_tower3_x = tower_adj[2] ; // back middle tower + xc
    delta_tower3_y = -2 * (-COS_60 * delta_radius);  
    */

    delta_tower1_x = (delta_radius + tower_adj[3]) * cos((210 + tower_adj[0]) * PI/180); // front left tower
    delta_tower1_y = (delta_radius + tower_adj[3]) * sin((210 + tower_adj[0]) * PI/180); 
    delta_tower2_x = (delta_radius + tower_adj[4]) * cos((330 + tower_adj[1]) * PI/180); // front right tower
    delta_tower2_y = (delta_radius + tower_adj[4]) * sin((330 + tower_adj[1]) * PI/180); 
    delta_tower3_x = (delta_radius + tower_adj[5]) * cos((90 + tower_adj[2]) * PI/180);  // back middle tower
    delta_tower3_y = (delta_radius + tower_adj[5]) * sin((90 + tower_adj[2]) * PI/180); 
  }

  void deploy_z_probe()
  {
    feedrate = homing_feedrate[X_AXIS];
    destination[X_AXIS] = z_probe_deploy_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_deploy_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_deploy_start_location[Z_AXIS];
    prepare_move_raw();

    feedrate = homing_feedrate[X_AXIS]/10;
    destination[X_AXIS] = z_probe_deploy_end_location[X_AXIS];
    destination[Y_AXIS] = z_probe_deploy_end_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_deploy_end_location[Z_AXIS];
    prepare_move_raw();

    feedrate = homing_feedrate[X_AXIS];
    destination[X_AXIS] = z_probe_deploy_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_deploy_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_deploy_start_location[Z_AXIS];
    prepare_move_raw();
    st_synchronize();
  }

  void retract_z_probe()
  {
    feedrate = homing_feedrate[X_AXIS];
    destination[Z_AXIS] = 50;
    prepare_move_raw();

    destination[X_AXIS] = z_probe_retract_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_retract_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_retract_start_location[Z_AXIS];
    prepare_move();
    prepare_move_raw();

    // Move the nozzle below the print surface to push the probe up.
    feedrate = homing_feedrate[Z_AXIS]/10;
    destination[X_AXIS] = z_probe_retract_end_location[X_AXIS];
    destination[Y_AXIS] = z_probe_retract_end_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_retract_end_location[Z_AXIS];
    prepare_move_raw();

    feedrate = homing_feedrate[Z_AXIS];
    destination[X_AXIS] = z_probe_retract_start_location[X_AXIS];
    destination[Y_AXIS] = z_probe_retract_start_location[Y_AXIS];
    destination[Z_AXIS] = z_probe_retract_start_location[Z_AXIS];
    prepare_move_raw();
    st_synchronize();
  }

  float z_probe()
  {
    feedrate = homing_feedrate[X_AXIS];
    prepare_move_raw();
    st_synchronize();

    enable_endstops(true);
    float start_z = current_position[Z_AXIS];
    long start_steps = st_get_position(Z_AXIS);

    feedrate = probing_feedrate;
    destination[Z_AXIS] = -20;
    prepare_move_raw();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    enable_endstops(false);
    long stop_steps = st_get_position(Z_AXIS);
    
    saved_position[X_AXIS] = float((st_get_position(X_AXIS)) / axis_steps_per_unit[X_AXIS]);
    saved_position[Y_AXIS] = float((st_get_position(Y_AXIS)) / axis_steps_per_unit[Y_AXIS]);
    saved_position[Z_AXIS] = float((st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS]);

    float mm = start_z - float(start_steps - stop_steps) / axis_steps_per_unit[Z_AXIS];
    current_position[Z_AXIS] = mm;
    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);

    saved_position[X_AXIS] = float((st_get_position(X_AXIS)) / axis_steps_per_unit[X_AXIS]);
    saved_position[Y_AXIS] = float((st_get_position(Y_AXIS)) / axis_steps_per_unit[Y_AXIS]);
    saved_position[Z_AXIS] = float((st_get_position(Z_AXIS)) / axis_steps_per_unit[Z_AXIS]);

    feedrate = homing_feedrate[Z_AXIS];
    destination[Z_AXIS] = mm+2;
    prepare_move_raw();
    return mm;
  }

  void calibrate_print_surface(float z_offset)
  {
    float probe_bed_z, probe_z, probe_h, probe_l;
    int probe_count;
      
    for (int y = 3; y >= -3; y--)
    {
      int dir = y % 2 ? -1 : 1;
      for (int x = -3*dir; x != 4*dir; x += dir)
      {
        if (x*x + y*y < 11)
        {
          destination[X_AXIS] = AUTOLEVEL_GRID * x - z_probe_offset[X_AXIS];
          if (destination[X_AXIS]<X_MIN_POS) destination[X_AXIS]=X_MIN_POS;
          if (destination[X_AXIS]>X_MAX_POS) destination[X_AXIS]=X_MAX_POS;
          destination[Y_AXIS] = AUTOLEVEL_GRID * y - z_probe_offset[Y_AXIS];
          if (destination[Y_AXIS]<Y_MIN_POS) destination[Y_AXIS]=Y_MIN_POS;
          if (destination[Y_AXIS]>Y_MAX_POS) destination[Y_AXIS]=Y_MAX_POS;
          probe_count = 0;
          probe_z = -100;
          probe_h = -100;
          probe_l = 100;
          do
          {
            probe_bed_z = probe_z;
            probe_z = z_probe() + z_offset;
            if (probe_z > probe_h) probe_h = probe_z;
            if (probe_z < probe_l) probe_l = probe_z;
            probe_count ++;
          } while ((probe_z != probe_bed_z) and (probe_count < 21));

          bed_level[x+3][3-y] = probe_bed_z;
        }
        else
        {
          bed_level[x+3][3-y] = 0.0;
        }
      }
      // For unprobed positions just copy nearest neighbor.
      if (abs(y) >= 3)
      {
        bed_level[1][3-y] = bed_level[2][3-y];
        bed_level[5][3-y] = bed_level[4][3-y];
      }
      if (abs(y) >=2)
      {
        bed_level[0][3-y] = bed_level[1][3-y];
        bed_level[6][3-y] = bed_level[5][3-y];
      }
      // Print calibration results for manual frame adjustment.
      for (int x = -3; x <= 3; x++)
      {
        SERIAL_PROTOCOL_F(bed_level[x+3][3-y], 3);
        SERIAL_PROTOCOLPGM(" ");
      }
      SERIAL_EOL;
    }
  }

  float probe_bed(float x, float y)
  {
    //Probe bed at specified location and return z height of bed
    float probe_bed_z, probe_z, probe_h, probe_l;
    int probe_count;
    //  feedrate = homing_feedrate[Z_AXIS];
    destination[X_AXIS] = x - z_probe_offset[X_AXIS];
    if (destination[X_AXIS]<X_MIN_POS) destination[X_AXIS]=X_MIN_POS;
    if (destination[X_AXIS]>X_MAX_POS) destination[X_AXIS]=X_MAX_POS;
    destination[Y_AXIS] = y - z_probe_offset[Y_AXIS];
    if (destination[Y_AXIS]<Y_MIN_POS) destination[Y_AXIS]=Y_MIN_POS;
    if (destination[Y_AXIS]>Y_MAX_POS) destination[Y_AXIS]=Y_MAX_POS;
    destination[Z_AXIS] = bed_level_c - z_probe_offset[Z_AXIS] + 3;
    prepare_move();
    st_synchronize();

    probe_count = 0;
    probe_z = -100;
    probe_h = -100;
    probe_l = 100;
    do
    {
      probe_bed_z = probe_z;
      probe_z = z_probe() + z_probe_offset[Z_AXIS];
      if (probe_z > probe_h) probe_h = probe_z;
      if (probe_z < probe_l) probe_l = probe_z;
      probe_count ++;
    } while ((probe_z != probe_bed_z) and (probe_count < 21));

    return probe_bed_z;
  }

  float z_probe_accuracy()
  {
    //Perform z-probe accuracy test
    float probe_h[7];
    float probe_l[7];
    float range_h = 0, range_l = 0;

    for(int x=0; x < 7; x++)
    {
      probe_h[x] = -100;
      probe_l[x] = 100;
    }
    
    // probe test loop  
    for(int x=0; x<3; x++)
    {
      bed_probe_all();

      if (bed_level_c > probe_h[0]) probe_h[0] = bed_level_c;
      if (bed_level_c < probe_l[0]) probe_l[0] = bed_level_c;
      if (bed_level_z > probe_h[1]) probe_h[1] = bed_level_z;
      if (bed_level_z < probe_l[1]) probe_l[1] = bed_level_z;
      if (bed_level_oy > probe_h[2]) probe_h[2] = bed_level_oy;
      if (bed_level_oy < probe_l[2]) probe_l[2] = bed_level_oy;
      if (bed_level_x > probe_h[3]) probe_h[3] = bed_level_x;
      if (bed_level_x < probe_l[3]) probe_l[3] = bed_level_x;
      if (bed_level_oz > probe_h[4]) probe_h[4] = bed_level_oz;
      if (bed_level_oz < probe_l[4]) probe_l[4] = bed_level_oz;
      if (bed_level_y > probe_h[5]) probe_h[5] = bed_level_y;
      if (bed_level_y < probe_l[5]) probe_l[5] = bed_level_y;
      if (bed_level_ox > probe_h[6]) probe_h[6] = bed_level_ox;
      if (bed_level_ox < probe_l[6]) probe_l[6] = bed_level_ox;
    }
    for(int x=0; x < 7; x++)
    {
      if (probe_h[x] - probe_l[x] > range_h) range_h = probe_h[x] - probe_l[x];
      if (probe_h[x] - probe_l[x] < range_l) range_l = probe_h[x] - probe_l[x];
    }
    return range_h - range_l;
  }

  void bed_probe_all()
  {
    //Probe all bed positions & store carriage positions
    bed_level_c = probe_bed(0.0, 0.0);      
    save_carriage_positions(0);
    bed_level_z = probe_bed(0.0, bed_radius);
    save_carriage_positions(1);
    bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
    save_carriage_positions(2);
    bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
    save_carriage_positions(3);
    bed_level_oz = probe_bed(0.0, -bed_radius);
    save_carriage_positions(4);
    bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);
    save_carriage_positions(5);
    bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
    save_carriage_positions(6);    
  }
    
  void calibration_report()
  {
    //Display Report
    

    SerialUSB.println("bed level");
	SerialUSB.print("bed_level_x:");
	SerialUSB.println(bed_level_x, 4);
    SerialUSB.print("bed_level_y:");
	SerialUSB.println(bed_level_y, 4);
	SerialUSB.print("bed_level_z:");
	SerialUSB.println(bed_level_z, 4);
	SerialUSB.print("bed_level_ox:");
	SerialUSB.println(bed_level_ox, 4);
	SerialUSB.print("bed_level_oy:");
    SerialUSB.println(bed_level_oy, 4);
    SerialUSB.print("bed_level_oz:");
    SerialUSB.println(bed_level_oz, 4);
	SerialUSB.print("bed_level_c:");
    SerialUSB.println(bed_level_c, 4);

	SerialUSB.println("Endstop");
	SerialUSB.print("X:");
    SerialUSB.print(endstop_adj[0]);
    SerialUSB.print(" Y:");
    SerialUSB.print(endstop_adj[1]);
	SerialUSB.print(" Z:");
    SerialUSB.println(endstop_adj[2]);


	SerialUSB.println("tower");
    SerialUSB.print("A:");
    SerialUSB.print(tower_adj[0]);
    SerialUSB.print(" B:");
    SerialUSB.print(tower_adj[1]);
	SerialUSB.print(" C:");
    SerialUSB.print(tower_adj[2]);
	SerialUSB.print(" I:");
    SerialUSB.print(tower_adj[3]);
    SerialUSB.print(" J:");
    SerialUSB.print(tower_adj[4]);
	SerialUSB.print(" K:");
    SerialUSB.println(tower_adj[5]);    

    SerialUSB.println("Delta Radius: ");
    SerialUSB.println(delta_radius, 4);
    

    SerialUSB.println("X-Tower\t\tY-Tower\t\tDiag Rod: ");
    SerialUSB.println(delta_diagonal_rod, 4);
    SERIAL_EOL;
  }

  void save_carriage_positions(int position_num)
  {
    for(int8_t i=0; i < NUM_AXIS; i++)
    {
      saved_positions[position_num][i] = saved_position[i];    
    }
  }

  void home_delta_axis()
  {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    refresh_cmd_timeout();

    enable_endstops(true);

    for(int8_t i=0; i < NUM_AXIS; i++)
    {
      destination[i] = current_position[i];
    }
    feedrate = 0.0;
    // Move all carriages up together until the first endstop is hit.
    current_position[X_AXIS] = 0;
    current_position[Y_AXIS] = 0;
    current_position[Z_AXIS] = 0;
    sync_plan_position();

    destination[X_AXIS] = 3 * max_length[Z_AXIS];
    destination[Y_AXIS] = 3 * max_length[Z_AXIS];
    destination[Z_AXIS] = 3 * max_length[Z_AXIS];
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    current_position[X_AXIS] = destination[X_AXIS];
    current_position[Y_AXIS] = destination[Y_AXIS];
    current_position[Z_AXIS] = destination[Z_AXIS];

    // take care of back off and rehome now we are all at the top
    HOMEAXIS(X);
    HOMEAXIS(Y);
    HOMEAXIS(Z);

    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);   

    #ifdef ENDSTOPS_ONLY_FOR_HOMING
      enable_endstops(false);
    #endif

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    refresh_cmd_timeout();
    endstops_hit_on_purpose(); // clear endstop hit flags
  }

  void prepare_move_raw()
  {
    refresh_cmd_timeout();
    calculate_delta(destination);
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder, active_driver);
    for(int8_t i=0; i < NUM_AXIS; i++)
    {
      current_position[i] = destination[i];
    }
  }

  void calculate_delta(float cartesian[3]) 
  {
  
    delta[X_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
                         - sq(delta_tower1_x-cartesian[X_AXIS])
                         - sq(delta_tower1_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[Y_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
                         - sq(delta_tower2_x-cartesian[X_AXIS])
                         - sq(delta_tower2_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
    delta[Z_AXIS] = sqrt(DELTA_DIAGONAL_ROD_2
                         - sq(delta_tower3_x-cartesian[X_AXIS])
                         - sq(delta_tower3_y-cartesian[Y_AXIS])
                         ) + cartesian[Z_AXIS];
#if 0
	SerialUSB.print(" X: ");
	SerialUSB.print(delta[X_AXIS]);
	SerialUSB.print(" Y: ");
	SerialUSB.print(delta[Y_AXIS]);
	SerialUSB.print(" Z: ");
	SerialUSB.println(delta[Z_AXIS]);
#endif	
  }

 //aven_0817
 void calculate_delta_tower(float r, float delta_tower[])
 { 

	DELTA_DIAGONAL_ROD_2 = delta_diagonal_rod * delta_diagonal_rod;

	delta_tower[0] = (r + tower_adj[3]) * cos((210 + tower_adj[0]) * M_PI/180); // front left tower
	delta_tower[1] = (r + tower_adj[3]) * sin((210 + tower_adj[0]) * M_PI/180); 
	delta_tower[2] = (r + tower_adj[4]) * cos((330 + tower_adj[1]) * M_PI/180); // front right tower
	delta_tower[3] = (r + tower_adj[4]) * sin((330 + tower_adj[1]) * M_PI/180); 
	delta_tower[4] = (r + tower_adj[5]) * cos((90 + tower_adj[2]) * M_PI/180);  // back middle tower
	delta_tower[5] = (r + tower_adj[5]) * sin((90 + tower_adj[2]) * M_PI/180); 
  }

  void calculate_cartesian_position( float actuator_mm[], float cartesian_mm[], float r)
  {
	float delta_tower[6];

	calculate_delta_tower(r, delta_tower);

	Vector3 tower1( delta_tower[0], delta_tower[1], actuator_mm[X_AXIS] );
	Vector3 tower2( delta_tower[2], delta_tower[3], actuator_mm[Y_AXIS] );
	Vector3 tower3( delta_tower[4], delta_tower[5], actuator_mm[Z_AXIS] );

	Vector3 s12 = tower1.sub(tower2);
	Vector3 s23 = tower2.sub(tower3);
	Vector3 s13 = tower1.sub(tower3);

	Vector3 normal = s12.cross(s23);

	float magsq_s12 = s12.magsq();
	float magsq_s23 = s23.magsq();
	float magsq_s13 = s13.magsq();

	float inv_nmag_sq = 1.0F / normal.magsq();
	float q = 0.5F * inv_nmag_sq;

	float a = q * magsq_s23 * s12.dot(s13);
	float b = q * magsq_s13 * s12.dot(s23) * -1.0F; // negate because we use s12 instead of s21
	float c = q * magsq_s12 * s13.dot(s23);

	Vector3 circumcenter( delta_tower[0] * a + delta_tower[2] * b + delta_tower[4] * c,
		delta_tower[1] * a + delta_tower[3] * b + delta_tower[5] * c,
		actuator_mm[X_AXIS] * a + actuator_mm[Y_AXIS] * b + actuator_mm[Z_AXIS] * c );

	float r_sq = 0.5F * q * magsq_s12 * magsq_s23 * magsq_s13;
	//float dist = sqrtf(inv_nmag_sq * (arm_length_squared - r_sq));
	float dist = sqrt(inv_nmag_sq * (DELTA_DIAGONAL_ROD_2 - r_sq));


	Vector3 cartesianln = circumcenter.sub(normal.mul(dist));

	cartesian_mm[X_AXIS] = cartesianln[0];
	cartesian_mm[Y_AXIS] = cartesianln[1];
	cartesian_mm[Z_AXIS] = cartesianln[2];
	;

  }

  void calculate_delta_position(float cartesian[3], float actuator_mm[], float r) 
  {

    float delta_tower[6];
	calculate_delta_tower(r, delta_tower);
	
	actuator_mm[0] = sqrt(DELTA_DIAGONAL_ROD_2
		- sq(delta_tower[0]-cartesian[0]) 
		- sq(delta_tower[1]-cartesian[1])
		) + cartesian[2];

	actuator_mm[1] = sqrt(DELTA_DIAGONAL_ROD_2
		- sq(delta_tower[2]-cartesian[0])
		- sq(delta_tower[3]-cartesian[1])
		) + cartesian[2];

	actuator_mm[2] = sqrt(DELTA_DIAGONAL_ROD_2
		- sq(delta_tower[4]-cartesian[0])
		- sq(delta_tower[5]-cartesian[1])
		) + cartesian[2];
	
  }

  void error_simulation(float p0[], float p1[], float error[])
  {
	float temp[3];
	calculate_delta_position(p0, temp, delta_radius);
	for (int i = 0; i < 3; i++) temp[i] += error[i];
	calculate_cartesian_position(temp, p1, delta_radius + error[3]);
  }

  int calculate_error(float p[][3], float err[], int r_en, int h_en)
  {


	float temp[4][3];
	float error[5] = {0};
	for(int i = 0; i < 4; i++) error_simulation(p[i], temp[i], error);
	int flag = 0;
	unsigned int count = 0;
	do
	{
		flag = 0;
		for(int i = 0; i < 3; i++)
		{
			float a = temp[i][2] - temp[(i + 1) % 3][2];
			float b = temp[i][2] - temp[(i + 2) % 3][2];

			if(a < -0.001 || b < -0.001)
			{
				error[i] += 0.001;
				for(int i = 0; i < 4; i++) error_simulation(p[i], temp[i], error);
				flag++;
			}
		}


		float c = 0;
		if (r_en) c = temp[3][2] - temp[0][2];

		if(c < -0.001)
		{
			error[3] += 0.001;
			for(int i = 0; i < 4; i++) error_simulation(p[i], temp[i], error);
			flag++;
		}
		else if(c > 0.001)
		{
			error[3] -= 0.001;
			for(int i = 0; i < 4; i++) error_simulation(p[i], temp[i], error);
			flag++;
		}
		if (count > 25530)
		{
			return 0;
		}
		count++;
	}while (flag);

	if (h_en) error[4] -= temp[3][2];

	//cout << "Count:" << count << endl;

	for (int i = 0; i < 5; i++) err[i] += error[i];
	float min = err[0];
	for (int i = 1; i < 3; i++)
	{
		if (err[i] < min) min = err[i];
	}
	for (int i = 0; i < 3; i++) err[i] -= min;
	//cout << "M666X" << -1 * err[0] << "Y" << -1 * err[1] << "Z" <<  -1 * err[2] << "R" << err[3] << "H" << err[4] << endl;


	return 1;



  }

  //aven_0817
  

  // Adjust print surface height by linear interpolation over the bed_level array.
  void adjust_delta(float cartesian[3])
  {
    float grid_x = max(-2.999, min(2.999, cartesian[X_AXIS] / AUTOLEVEL_GRID));
    float grid_y = max(-2.999, min(2.999, cartesian[Y_AXIS] / AUTOLEVEL_GRID));
    int floor_x = floor(grid_x);
    int floor_y = floor(grid_y);
    float ratio_x = grid_x - floor_x;
    float ratio_y = grid_y - floor_y;
    float z1 = bed_level[floor_x+3][floor_y+3];
    float z2 = bed_level[floor_x+3][floor_y+4];
    float z3 = bed_level[floor_x+4][floor_y+3];
    float z4 = bed_level[floor_x+4][floor_y+4];
    float left = (1-ratio_y)*z1 + ratio_y*z2;
    float right = (1-ratio_y)*z3 + ratio_y*z4;
    float offset = (1-ratio_x)*left + ratio_x*right;

    delta[X_AXIS] += offset;
    delta[Y_AXIS] += offset;
    delta[Z_AXIS] += offset;

#if 0
	SerialUSB.print(" aX: ");
	SerialUSB.print(delta[X_AXIS]);
	SerialUSB.print(" aY: ");
	SerialUSB.print(delta[Y_AXIS]);
	SerialUSB.print(" aZ: ");
	SerialUSB.println(delta[Z_AXIS]);
#endif
    /*
    SERIAL_ECHOPGM("grid_x="); SERIAL_ECHO(grid_x);
    SERIAL_ECHOPGM(" grid_y="); SERIAL_ECHO(grid_y);
    SERIAL_ECHOPGM(" floor_x="); SERIAL_ECHO(floor_x);
    SERIAL_ECHOPGM(" floor_y="); SERIAL_ECHO(floor_y);
    SERIAL_ECHOPGM(" ratio_x="); SERIAL_ECHO(ratio_x);
    SERIAL_ECHOPGM(" ratio_y="); SERIAL_ECHO(ratio_y);
    SERIAL_ECHOPGM(" z1="); SERIAL_ECHO(z1);
    SERIAL_ECHOPGM(" z2="); SERIAL_ECHO(z2);
    SERIAL_ECHOPGM(" z3="); SERIAL_ECHO(z3);
    SERIAL_ECHOPGM(" z4="); SERIAL_ECHO(z4);
    SERIAL_ECHOPGM(" left="); SERIAL_ECHO(left);
    SERIAL_ECHOPGM(" right="); SERIAL_ECHO(right);
    SERIAL_ECHOPGM(" offset="); SERIAL_ECHOLN(offset);
    */
  }

//aven_0813
  void apply_endstop_adjustment(float x_endstop, float y_endstop, float z_endstop) {

    memcpy(saved_endstop_adj, endstop_adj, sizeof(saved_endstop_adj));
    endstop_adj[X_AXIS] += x_endstop;
    endstop_adj[Y_AXIS] += y_endstop;
    endstop_adj[Z_AXIS] += z_endstop;

    calculate_delta(current_position);
    plan_set_position(delta[X_AXIS] - (endstop_adj[X_AXIS] - saved_endstop_adj[X_AXIS]) , delta[Y_AXIS] - (endstop_adj[Y_AXIS] - saved_endstop_adj[Y_AXIS]), delta[Z_AXIS] - (endstop_adj[Z_AXIS] - saved_endstop_adj[Z_AXIS]), current_position[E_AXIS]);  
    st_synchronize();
  }

  void adj_endstops()
  {
    boolean x_done = false;
    boolean y_done = false;
    boolean z_done = false;
    float prv_bed_level_x, prv_bed_level_y, prv_bed_level_z;

    do {
      bed_level_z = probe_bed(0.0, bed_radius);
      bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
      bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);

      apply_endstop_adjustment(bed_level_x, bed_level_y, bed_level_z);

      SerialUSB.print("x:");
	  SerialUSB.print(bed_level_x, 4);
      SerialUSB.print(" (adj:");
	  SerialUSB.print(endstop_adj[0], 4);
	  SerialUSB.print(") y:");
	  SerialUSB.print(bed_level_y, 4);
	  SerialUSB.print(" (adj:");
	  SerialUSB.print(endstop_adj[1], 4);
	  SerialUSB.print(") z:");
	  SerialUSB.print(bed_level_z, 4);
	  SerialUSB.print(" (adj:");
	  SerialUSB.print(endstop_adj[2], 4);
	  SerialUSB.println(" (adj:");

	  #if 0
      ECHO_SMV(DB, "x:", bed_level_x, 4);
      ECHO_MV(" (adj:", endstop_adj[0], 4);
      ECHO_MV(") y:", bed_level_y, 4);
      ECHO_MV(" (adj:", endstop_adj[1], 4);
      ECHO_MV(") z:", bed_level_z, 4);
      ECHO_MV(" (adj:", endstop_adj[2], 4);
      ECHO_EM(")");
      #endif

      if ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)) {
        x_done = true;
        SerialUSB.println("X=OK ");
		//ECHO_SM(DB, "X=OK ");
      }
      else {
        x_done = false;
		SerialUSB.println("X=ERROR ");
        //ECHO_SM(DB, "X=ERROR ");
      }

      if ((bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)) {
        y_done = true;
		SerialUSB.println("Y=OK ");
        //ECHO_M("Y=OK ");
      }
      else {
        y_done = false;
		SerialUSB.println("Y=ERROR ");
        //ECHO_M("Y=ERROR ");
      }

      if ((bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)) {
        z_done = true;
		SerialUSB.println("Z=OK ");
        //ECHO_EM("Z=OK");
      }
      else {
        z_done = false;
		SerialUSB.println("Z=ERROR");
        //ECHO_EM("Z=ERROR");
      }
    } while (((x_done == false) or (y_done == false) or (z_done == false)));

    float high_endstop  = 0;
    float low_endstop   = 0;
    for(int x = 0; x < 3; x++) {
      if (endstop_adj[x] > high_endstop) high_endstop = endstop_adj[x];
      if (endstop_adj[x] < low_endstop) low_endstop = endstop_adj[x];
    }

    if (high_endstop > 0) {
      SerialUSB.print("Reducing Build height by ");
	  SerialUSB.println(high_endstop);
	  //ECHO_LMV(DB, "Reducing Build height by ", high_endstop);
      for(int8_t i = 0; i < 3; i++) {
        endstop_adj[i] -= high_endstop;
      }
      max_pos[Z_AXIS] -= high_endstop;
      set_delta_constants();
    }

    bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
  }

  int adj_deltaradius()
  { 
    float adj_r;
    float prev_c;
    int c_nochange_count = 0;
    float nochange_r;

    bed_level_c = probe_bed(0.0, 0.0);

    if ((bed_level_c >= -ac_prec / 2) and (bed_level_c <= ac_prec / 2)) {
	  SerialUSB.println("Delta Radius OK");	
      //ECHO_LM(DB, "Delta Radius OK");
      return 0;
    }
    else {
	  SerialUSB.println("Adjusting Delta Radius");
      //ECHO_LM(DB, "Adjusting Delta Radius");
      //set inital direction and magnitude for delta radius adjustment
      adj_r = 0.1;
      if (bed_level_c > 0) adj_r = -0.1;

      bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];

      do {
        delta_radius += adj_r;
        set_delta_constants();

        prev_c = bed_level_c;
        bed_level_c = probe_bed(0.0, 0.0);

        //Show progress

		SerialUSB.print("r:");
		SerialUSB.print(delta_radius, 4);
		SerialUSB.print(" (adj:");
		SerialUSB.print(adj_r, 4);
		SerialUSB.print(") c:");
		SerialUSB.println(bed_level_c, 4);
		
        //ECHO_SMV(DB, "r:", delta_radius, 4);
        //ECHO_MV(" (adj:", adj_r, 4);
        //ECHO_EMV(") c:",bed_level_c, 4);

        //Adjust delta radius
        if (((adj_r > 0) and (bed_level_c < prev_c)) or ((adj_r < 0) and (bed_level_c > prev_c))) adj_r = -(adj_r / 2);

        //Count iterations with no change to c probe point
        if (bed_level_c == prev_c) c_nochange_count ++;
        if (c_nochange_count == 1) nochange_r = delta_radius;

      } while(((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)) and (c_nochange_count < 3));

      if (c_nochange_count > 0) {
        delta_radius = nochange_r;
        set_delta_constants();
        bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
      }
      return 1;
    }
  }


  void adj_tower_delta(int tower)
  {
    float adj_val = 0;
    float adj_mag = 0.2;
    float adj_prv;

    do {

      tower_adj[tower - 1] += adj_val;
      set_delta_constants();

      if ((tower == 1) or (tower == 3)) bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
      if ((tower == 1) or (tower == 2)) bed_level_oz = probe_bed(0.0, -bed_radius);
      if ((tower == 2) or (tower == 3)) bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);

      adj_prv = adj_val;
      adj_val = 0;

      if (tower == 1) {
        if (bed_level_oy < bed_level_oz) adj_val = adj_mag;
        if (bed_level_oy > bed_level_oz) adj_val = -adj_mag;
      }

      if (tower == 2) {
        if (bed_level_oz < bed_level_ox) adj_val = adj_mag;
        if (bed_level_oz > bed_level_ox) adj_val = -adj_mag;
      }

      if (tower == 3) {
        if (bed_level_ox < bed_level_oy) adj_val = adj_mag;
        if (bed_level_ox > bed_level_oy) adj_val = -adj_mag;
      }
         
      if ((adj_val > 0) and (adj_prv < 0)) {
        adj_mag = adj_mag / 2;
        adj_val = adj_mag;
      }

      if ((adj_val < 0) and (adj_prv > 0)) {
        adj_mag = adj_mag / 2;
        adj_val = -adj_mag;
      }

      //Show Adjustments made
      if (tower == 1) {

		SerialUSB.print("oy:");
		SerialUSB.print(bed_level_oy, 4);
		SerialUSB.print("oz:");
		SerialUSB.println(bed_level_oz, 4);
		
        //ECHO_SMV(DB, "oy:", bed_level_oy, 4);
        //ECHO_MV(" oz:", bed_level_oz, 4);
      }

      if (tower == 2) {

		SerialUSB.print("ox:");
		SerialUSB.print(bed_level_ox, 4);
		SerialUSB.print("oz:");
		SerialUSB.println(bed_level_oz, 4);
		
        //ECHO_SMV(DB, "ox:", bed_level_ox, 4);
        //ECHO_MV(" oz:", bed_level_oz, 4);
      }

      if (tower == 3) {
     
	    SerialUSB.print("ox:");
		SerialUSB.print(bed_level_ox, 4);
		SerialUSB.print("oy:");
		SerialUSB.println(bed_level_oy, 4);
		
        //ECHO_SMV(DB, "ox:", bed_level_ox, 4);
        //ECHO_MV(" oy:", bed_level_oy, 4);
      }

      SerialUSB.print(" tower delta adj:");
	  SerialUSB.println(adj_val, 5);
		
      //ECHO_EMV(" tower delta adj:", adj_val, 5);
    } while(adj_val != 0);
  }

  
  void adj_tower_radius(int tower)
  {
    boolean done,t1_done,t2_done,t3_done;
    int nochange_count;
    float target, prev_target, prev_bed_level;
    float temp, adj_target;

    //Set inital tower adjustment values
    adj_t1_Radius = 0;
    adj_t2_Radius = 0;
    adj_t3_Radius = 0;
    nochange_count = 0;

    if ((tower == 1) and (adj_t1_Radius == 0)) {
      target = (bed_level_oy + bed_level_oz) / 2;
      temp = (bed_level_ox - target) / 2;
      adj_target = target + temp;
      if (bed_level_ox < adj_target) adj_t1_Radius = -0.4;
      if (bed_level_ox > adj_target) adj_t1_Radius = 0.4;
    }
    if ((tower == 2) and (adj_t2_Radius == 0)) {
      target = (bed_level_ox + bed_level_oz) / 2;
      temp = (bed_level_oy - target) / 2;
      adj_target = target + temp;
      if (bed_level_oy < adj_target) adj_t2_Radius = -0.4;
      if (bed_level_oy > adj_target) adj_t2_Radius = 0.4;
    }
    if ((tower == 3) and (adj_t3_Radius == 0)) {
      target = (bed_level_oy + bed_level_ox) / 2;
      temp = (bed_level_oz - target) / 2;
      adj_target = target + temp;
      if (bed_level_oz < adj_target) adj_t3_Radius = -0.4; //0.4;
      if (bed_level_oz > adj_target) adj_t3_Radius = 0.4; //-0.4;
    }

    do {
      tower_adj[3] += adj_t1_Radius;
      tower_adj[4] += adj_t2_Radius;
      tower_adj[5] += adj_t3_Radius;
      set_delta_constants();

      //done = false;
      t1_done = false;
      t2_done = false;
      t3_done = false;
      if (tower == 1) {
        t2_done = true;
        t3_done = true;
        prev_target = adj_target;
        prev_bed_level = bed_level_ox;

        bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oz = probe_bed(0.0, -bed_radius);

        target = (bed_level_oy + bed_level_oz) / 2;
        temp = (bed_level_ox - target) / 2;
        adj_target = target + temp;
        if (((bed_level_ox < adj_target) and (adj_t1_Radius > 0)) or ((bed_level_ox > adj_target) and (adj_t1_Radius < 0))) adj_t1_Radius = -(adj_t1_Radius / 2);
        if (bed_level_ox == adj_target) t1_done = true;
        if ((bed_level_ox + 0.0001 > prev_bed_level) and (bed_level_ox - 0.0001 < prev_bed_level) and (adj_target + 0.0001 > prev_target) and (adj_target - 0.0001 < prev_target)) nochange_count ++;
        if (nochange_count > 1) {
		  SerialUSB.println("Stuck in Loop.. Exiting");	
          //ECHO_LM(DB, "Stuck in Loop.. Exiting");
          t1_done = true;
        }

        SerialUSB.print("target:");
		SerialUSB.print(adj_target, 6);
		SerialUSB.print(" ox:");
		SerialUSB.print(bed_level_ox, 6);
		SerialUSB.print(" tower radius adj:");
		SerialUSB.println(tower_adj[3], 6);
		
		
        //ECHO_SMV(DB, "target:", adj_target, 6);
        //ECHO_MV(" ox:", bed_level_ox, 6);
        //ECHO_MV(" tower radius adj:", tower_adj[3], 6);
        if (t1_done == true)
			SerialUSB.println(" done:true");
			//ECHO_EM(" done:true"); 
		else 
			SerialUSB.println(" done:false");
			//ECHO_EM(" done:false");
      }

      if (tower == 2) {
        t1_done = true;
        t3_done = true;
        prev_target = adj_target;
        prev_bed_level = bed_level_oy;

        bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oz = probe_bed(0.0, -bed_radius);

        target = (bed_level_ox + bed_level_oz) /2;
        temp = (bed_level_oy - target) / 2;
        adj_target = target + temp;
        if (((bed_level_oy < adj_target) and (adj_t2_Radius > 0)) or ((bed_level_oy > adj_target) and (adj_t2_Radius < 0))) adj_t2_Radius = -(adj_t2_Radius / 2);
        if (bed_level_oy == adj_target) t2_done = true;
        if ((bed_level_oy + 0.0001 > prev_bed_level) and (bed_level_oy - 0.0001 < prev_bed_level) and (adj_target + 0.0001 > prev_target) and (adj_target - 0.0001 < prev_target)) nochange_count ++;
        if (nochange_count > 1) {
		  SerialUSB.println("Stuck in Loop.. Exiting");	
          //ECHO_LM(DB, "Stuck in Loop.. Exiting");
          t2_done = true;
        }

        SerialUSB.print("target:");
		SerialUSB.print(adj_target, 6);
		SerialUSB.print(" oy:");
		SerialUSB.print(bed_level_oy, 6);
		SerialUSB.print(" tower radius adj:");
		SerialUSB.println(tower_adj[4], 6);
		
        //ECHO_SMV(DB, "target:", adj_target, 6);
        //ECHO_MV(" oy:", bed_level_oy, 6);
        //ECHO_MV(" tower radius adj:", tower_adj[4], 6);
        
        if (t2_done == true)
			SerialUSB.println(" done:true");
			//ECHO_EM(" done:true"); 
		else
			SerialUSB.println(" done:false");
			//ECHO_EM(" done:false");
      }

      if (tower == 3) {
        t1_done = true;
        t2_done = true;
        prev_target = adj_target;
        prev_bed_level = bed_level_oz;

        bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oz = probe_bed(0.0, -bed_radius);

        target = (bed_level_oy + bed_level_ox) / 2;
        temp = (bed_level_oz - target) / 2;
        adj_target = target + temp;
        if (((bed_level_oz < adj_target) and (adj_t3_Radius > 0)) or ((bed_level_oz > adj_target) and (adj_t3_Radius < 0))) adj_t3_Radius = -(adj_t3_Radius / 2);
        if (bed_level_oz == adj_target) t3_done = true;
        if ((bed_level_oz + 0.0001 > prev_bed_level) and (bed_level_oz - 0.0001 < prev_bed_level) and (adj_target + 0.0001 > prev_target) and (adj_target - 0.0001 < prev_target)) nochange_count ++;
        if (nochange_count > 1) {
		  SerialUSB.println("Stuck in Loop.. Exiting");	
          //ECHO_LM(DB, "Stuck in Loop.. Exiting");
          t3_done = true;
        }

		SerialUSB.print("target:");
		SerialUSB.print(adj_target, 6);
		SerialUSB.print(" oz:");
		SerialUSB.print(bed_level_oz, 6);
		SerialUSB.print(" tower radius adj:");
		SerialUSB.println(tower_adj[5], 6);
		
        //ECHO_SMV(DB, "target:", adj_target, 6);
        //ECHO_MV(" oz:", bed_level_oz, 6);
        //ECHO_MV(" tower radius adj:", tower_adj[5], 6);
        if (t3_done == true)
			SerialUSB.println(" done:true");
			//ECHO_EM(" done:true"); 
		else
			SerialUSB.println(" done:false");
			//ECHO_EM(" done:false");
      }

    } while ((t1_done == false) or (t2_done == false) or (t3_done == false));
  }

  float adj_diagrod_length()
  {
    float adj_val = 0;
    float adj_mag = 0.2;
    float adj_prv, target;
    float prev_diag_rod = delta_diagonal_rod;

    do {
      delta_diagonal_rod += adj_val;
      set_delta_constants();

      bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
      bed_level_oz = probe_bed(0.0, -bed_radius);
      bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
      bed_level_c = probe_bed(0.0, 0.0);

      target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;
      adj_prv = adj_val;
      adj_val = 0;

      if (bed_level_c - 0.005 < target) adj_val = -adj_mag;
      if (bed_level_c + 0.005 > target) adj_val = adj_mag;

      if (((adj_val > 0) and (adj_prv < 0)) or ((adj_val < 0) and (adj_prv > 0))) {
        adj_val = adj_val / 2;
        adj_mag = adj_mag / 2;
      }

      if ((bed_level_c - 0.005 < target) and (bed_level_c + 0.005 > target)) adj_val = 0;

      //If adj magnatude is very small.. quit adjusting
      if ((abs(adj_val) < 0.001) and (adj_val != 0)) adj_val = 0;

      SerialUSB.print("target:");
      SerialUSB.print(target, 4);
	  SerialUSB.print(" c:");
	  SerialUSB.print(bed_level_c, 4);
	  SerialUSB.print(" adj:");
	  SerialUSB.print(adj_val, 5);
	  
      //ECHO_SMV(DB, "target:", target, 4);
      //ECHO_MV(" c:", bed_level_c, 4);
      //ECHO_EMV(" adj:", adj_val, 5);
    } while(adj_val != 0);
    return (delta_diagonal_rod - prev_diag_rod);
  }

  
  int fix_tower_errors()
  {
    boolean t1_err, t2_err, t3_err;
    boolean xy_equal, xz_equal, yz_equal;
    float saved_tower_adj[6];
    int err_tower;
    float low_diff, high_diff;
    float x_diff, y_diff, z_diff;
    float xy_diff, yz_diff, xz_diff;
    float low_opp, high_opp;

    for (int8_t i = 0; i < 6; i++) saved_tower_adj[i] = tower_adj[i];

    err_tower = 0;

    x_diff = abs(bed_level_x - bed_level_ox);
    high_diff = x_diff;
    y_diff = abs(bed_level_y - bed_level_oy);
    if (y_diff > high_diff) high_diff = y_diff;
    z_diff = abs(bed_level_z - bed_level_oz);
    if (z_diff > high_diff) high_diff = z_diff;

    if (x_diff <= ac_prec) t1_err = false; else t1_err = true;
    if (y_diff <= ac_prec) t2_err = false; else t2_err = true;
    if (z_diff <= ac_prec) t3_err = false; else t3_err = true;

    SerialUSB.print("x_diff = ");
	SerialUSB.print(x_diff, 5);
	SerialUSB.print("y_diff = ");
	SerialUSB.print(y_diff, 5);
	SerialUSB.print("z_diff = ");
    SerialUSB.print(z_diff, 5);
	SerialUSB.print("high_diff = ");
	SerialUSB.println(high_diff, 5);

	#if 0
    ECHO_LMV(DB, "x_diff = ", x_diff, 5);
    ECHO_LMV(DB, "y_diff = ", y_diff, 5);
    ECHO_LMV(DB, "z_diff = ", z_diff, 5);
    ECHO_LMV(DB, "high_diff = ", high_diff, 5);
    #endif

    //Are all errors equal? (within defined precision)
    xy_equal = false;
    xz_equal = false;
    yz_equal = false;
    if (abs(x_diff - y_diff) <= ac_prec) xy_equal = true;
    if (abs(x_diff - z_diff) <= ac_prec) xz_equal = true;
    if (abs(y_diff - z_diff) <= ac_prec) yz_equal = true;

	SerialUSB.print("xy_equal = ");
    //ECHO_SM(DB, "xy_equal = ");
    if (xy_equal == true)
		SerialUSB.println("true");
		//ECHO_EM("true"); 
	else
		SerialUSB.println("false");
		//ECHO_EM("false");
		
	SerialUSB.print("xz_equal = ");
    //ECHO_SM(DB, "xz_equal = ");
    if (xz_equal == true)
		SerialUSB.println("true");
		//ECHO_EM("true"); 
	else 
        SerialUSB.println("false");
	    //ECHO_EM("false");
	    
	SerialUSB.print("yz_equal = ");    
    //ECHO_SM(DB, "yz_equal = ");
    if (yz_equal == true)
		SerialUSB.println("true");
		//ECHO_EM("true");
	else 
        SerialUSB.println("false");
	    //ECHO_EM("false");

    low_opp = bed_level_ox;
    high_opp = low_opp;
    if (bed_level_oy < low_opp) low_opp = bed_level_oy;
    if (bed_level_oy > high_opp) high_opp = bed_level_oy;
    if (bed_level_oz < low_opp) low_opp = bed_level_oz;
    if (bed_level_oz > high_opp) high_opp = bed_level_oz;

    SerialUSB.print("Opp Range = ");  
	SerialUSB.println(high_opp - low_opp, 5);
    //ECHO_LMV(DB, "Opp Range = ", high_opp - low_opp, 5);

    if (high_opp - low_opp  < ac_prec) {
	  SerialUSB.println("Opposite Points within Limits - Adjustment not required");	
      //ECHO_LM(DB, "Opposite Points within Limits - Adjustment not required");
      t1_err = false;
      t2_err = false;
      t3_err = false;
    }

    //All Towers have errors 
    if ((t1_err == true) and (t2_err == true) and (t3_err == true)) {
      if ((xy_equal == false) or (xz_equal == false) or (yz_equal == false)) {
        //Errors not equal .. select the tower that needs to be adjusted
        if (abs(high_diff - x_diff) < 0.00001) err_tower = 1;
        if (abs(high_diff - y_diff) < 0.00001) err_tower = 2;
        if (abs(high_diff - z_diff) < 0.00001) err_tower = 3;
		SerialUSB.print("Tower ");  
	    SerialUSB.print(err_tower);
		SerialUSB.println(" has largest error");
        //ECHO_SMV(DB, "Tower ", err_tower);
        //ECHO_EM(" has largest error");
      }
      if ((xy_equal == true) and (xz_equal == true) and (yz_equal == true)) {
	  	SerialUSB.println("All Towers Errors Equal");
        //ECHO_LM(DB, "All Towers Errors Equal");
        t1_err = false;
        t2_err = false;
        t3_err = false;
      }
    }

    //Two tower errors
    if ((t1_err == true) and (t2_err == true) and (t3_err == false)) err_tower = 3;
    if ((t1_err == true) and (t2_err == false) and (t3_err == true)) err_tower = 2;
    if ((t1_err == false) and (t2_err == true) and (t3_err == true)) err_tower = 1;

    //Single tower error
    if ((t1_err == true) and (t2_err == false) and (t3_err == false)) err_tower = 1;
    if ((t1_err == false) and (t2_err == true) and (t3_err == false)) err_tower = 2;
    if ((t1_err == false) and (t2_err == false) and (t3_err == true)) err_tower = 3;

    SerialUSB.print("t1:"); 
    //ECHO_SM(DB, "t1:");
    if (t1_err == true)
		SerialUSB.println("Err"); 
		//ECHO_M("Err"); 
	else 
		SerialUSB.println("OK");
		//ECHO_M("OK");

	SerialUSB.print(" t2:"); 	
    //ECHO_M(" t2:");
    if (t2_err == true)
		SerialUSB.println("Err"); 
		//ECHO_M("Err"); 
	else 
		SerialUSB.println("OK");
		//ECHO_M("OK");

	SerialUSB.print(" t3:"); 		
    //ECHO_M(" t3:");
    if (t3_err == true)
		SerialUSB.println("Err");
		//ECHO_M("Err"); 
	else
		SerialUSB.println("OK");
		//ECHO_M("OK");
		
    //ECHO_E;

    if (err_tower == 0) {
	  SerialUSB.println("Tower geometry OK");	
      //ECHO_LM(DB, "Tower geometry OK");
    }
    else {
      //If a tower has been adjusted previously.. continue to correct by adjusting that tower! (but only if the difference between the opp points is still large)
      if (high_opp - low_opp  > ac_prec * 2) {
        if ((tower_adj[0] != 0) or (tower_adj[3] != 0)) {
		  SerialUSB.println("Tower 1 has already been adjusted");		
          //ECHO_LM(DB, "Tower 1 has already been adjusted");
          err_tower = 1;
        }
        if ((tower_adj[1] != 0) or (tower_adj[4] != 0)) {
		  SerialUSB.println("Tower 2 has already been adjusted");		
          //ECHO_LM(DB, "Tower 2 has already been adjusted");
          err_tower = 2;
        }
        if ((tower_adj[2] != 0) or (tower_adj[5] != 0)) {
          SerialUSB.println("Tower 3 has already been adjusted");	
		  //ECHO_LM(DB, "Tower 3 has already been adjusted");
          err_tower = 3;
        }
      }
	  SerialUSB.print("Tower");
	  SerialUSB.print(int(err_tower));
	  SerialUSB.print(" Error: Adjusting");
      //ECHO_SMV(DB, "Tower", int(err_tower));
      //ECHO_EM(" Error: Adjusting");
      adj_tower_radius(err_tower);
      //adj_tower_delta(err_tower);
    }

    //Set return value to indicate if anything has been changed (0 = no change)
    int retval = 0;
    for (int8_t i = 0; i < 6; i++) if (saved_tower_adj[i] != tower_adj[i]) retval++;
    return retval;
  }

  //aven_0812
  void actuator_to_cartesian( float delta[3])
  {
    
    //Vector3 tower1( delta_tower1_x, delta_tower1_y, delta[X_AXIS]+endstop_adj[0] );
    //Vector3 tower2( delta_tower2_x, delta_tower2_y, delta[Y_AXIS]+endstop_adj[1] );
    //Vector3 tower3( delta_tower3_x, delta_tower3_y, delta[Z_AXIS]+endstop_adj[2] );

    //aven_0815
	Vector3 tower1( delta_tower1_x, delta_tower1_y, delta[X_AXIS] );
    Vector3 tower2( delta_tower2_x, delta_tower2_y, delta[Y_AXIS] );
    Vector3 tower3( delta_tower3_x, delta_tower3_y, delta[Z_AXIS] );

	Vector3 s12 = tower1.sub(tower2);
    Vector3 s23 = tower2.sub(tower3);
    Vector3 s13 = tower1.sub(tower3);

    Vector3 normal = s12.cross(s23);

    float magsq_s12 = s12.magsq();
    float magsq_s23 = s23.magsq();
    float magsq_s13 = s13.magsq();

    float inv_nmag_sq = 1.0F / normal.magsq();
    float q = 0.5F * inv_nmag_sq;

    float a = q * magsq_s23 * s12.dot(s13);
    float b = q * magsq_s13 * s12.dot(s23) * -1.0F; // negate because we use s12 instead of s21
    float c = q * magsq_s12 * s13.dot(s23);

	Vector3 circumcenter( delta_tower1_x * a + delta_tower2_x * b + delta_tower3_x * c,
                          delta_tower1_y * a + delta_tower2_y * b + delta_tower3_y * c,
                          delta[X_AXIS] * a + delta[Y_AXIS] * b + delta[Z_AXIS] * c );

    float r_sq = 0.5F * q * magsq_s12 * magsq_s23 * magsq_s13;
    //float dist = sqrtf(inv_nmag_sq * (arm_length_squared - r_sq));
    float dist = sqrt(inv_nmag_sq * (DELTA_DIAGONAL_ROD_2 - r_sq));
   

    Vector3 cartesianln = circumcenter.sub(normal.mul(dist));
            
    cartesian[X_AXIS] = ROUND(cartesianln[0], 4);
    cartesian[Y_AXIS] = ROUND(cartesianln[1], 4);
    cartesian[Z_AXIS] = ROUND(cartesianln[2], 4);

	//SerialUSB.print("X : ");
	//SerialUSB.print(cartesian[X_AXIS]);
	//SerialUSB.print(" Y : ");
	//SerialUSB.print(cartesian[Y_AXIS]);
	//SerialUSB.print(" Z : ");
	//SerialUSB.print(cartesian[Z_AXIS]);
	
  }
//aven_0812
#endif //DELTA

#ifdef IDLE_OOZING_PREVENT
  void IDLE_OOZING_retract(bool retracting)
  {  
    if (retracting && !IDLE_OOZING_retracted[active_extruder]) {
  	  //SERIAL_ECHOLN("RETRACT FOR OOZING PREVENT");
  	  destination[X_AXIS]=current_position[X_AXIS];
  	  destination[Y_AXIS]=current_position[Y_AXIS];
  	  destination[Z_AXIS]=current_position[Z_AXIS];
  	  destination[E_AXIS]=current_position[E_AXIS];
  	  current_position[E_AXIS]+=IDLE_OOZING_LENGTH/volumetric_multiplier[active_extruder];
  	  plan_set_e_position(current_position[E_AXIS]);
  	  float oldFeedrate = feedrate;
  	  feedrate=IDLE_OOZING_FEEDRATE*60;
  	  IDLE_OOZING_retracted[active_extruder]=true;
  	  prepare_move();
  	  feedrate = oldFeedrate;
    }
    else if(!retracting && IDLE_OOZING_retracted[active_extruder]) {
  	  //SERIAL_ECHOLN("EXTRUDE FOR OOZING PREVENT");
  	  destination[X_AXIS]=current_position[X_AXIS];
  	  destination[Y_AXIS]=current_position[Y_AXIS];
  	  destination[Z_AXIS]=current_position[Z_AXIS];
  	  destination[E_AXIS]=current_position[E_AXIS];
  	  current_position[E_AXIS]-=(IDLE_OOZING_LENGTH+IDLE_OOZING_RECOVER_LENGTH)/volumetric_multiplier[active_extruder];
  	  plan_set_e_position(current_position[E_AXIS]);
  	  float oldFeedrate = feedrate;
  	  feedrate=IDLE_OOZING_RECOVER_FEEDRATE * 60;
  	  IDLE_OOZING_retracted[active_extruder] = false;
  	  prepare_move();
      feedrate = oldFeedrate;
    }
  }
#endif

#ifdef FWRETRACT
  void retract(bool retracting, bool swapretract = false) {

    if (retracting == retracted[active_extruder]) return;

    float oldFeedrate = feedrate;

    set_destination_to_current();

    if (retracting) {

      feedrate = retract_feedrate * 60;
      current_position[E_AXIS] += (swapretract ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] -= retract_zlift;
        #ifdef DELTA
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        prepare_move();
      }
    }
    else {

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] += retract_zlift;
        #ifdef DELTA
          sync_plan_position_delta();
        #else
          sync_plan_position();
        #endif
        //prepare_move();
      }

      feedrate = retract_recover_feedrate * 60;
      float move_e = swapretract ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      plan_set_e_position(current_position[E_AXIS]);
      prepare_move();
    }

    feedrate = oldFeedrate;
    retracted[active_extruder] = retracting;

  } // retract()
#endif //FWRETRACT

#ifdef Z_PROBE_SLED

  #ifndef SLED_DOCKING_OFFSET
    #define SLED_DOCKING_OFFSET 0
  #endif

  //
  // Method to dock/undock a sled designed by Charles Bell.
  //
  // dock[in]     If true, move to MAX_X and engage the electromagnet
  // offset[in]   The additional distance to move to adjust docking location
  //
  static void dock_sled(bool dock, int offset=0) {
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
      return;
    }

    if (dock) {
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, current_position[Y_AXIS], current_position[Z_AXIS]);
      digitalWrite(SERVO0_PIN, LOW); // turn off magnet
    } else {
      float z_loc = current_position[Z_AXIS];
      if (z_loc < Z_RAISE_BEFORE_PROBING + 5) z_loc = Z_RAISE_BEFORE_PROBING;
      do_blocking_move_to(X_MAX_POS + SLED_DOCKING_OFFSET + offset, Y_PROBE_OFFSET_FROM_EXTRUDER, z_loc);
      digitalWrite(SERVO0_PIN, HIGH); // turn on magnet
    }
  }
#endif //Z_PROBE_SLED

inline void lcd_beep(int number_beep = 3) {
  #ifdef LCD_USE_I2C_BUZZER
    #if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
      for(int8_t i=0;i<3;i++) {
        lcd_buzz(1000/6,100);
      }
    #else
      for(int8_t i=0;i<number_beep;i++) {
        lcd_buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS,LCD_FEEDBACK_FREQUENCY_HZ);
      }
    #endif
  #elif defined(BEEPER) && BEEPER > -1
    SET_OUTPUT(BEEPER);
    #if !defined(LCD_FEEDBACK_FREQUENCY_HZ) || !defined(LCD_FEEDBACK_FREQUENCY_DURATION_MS)
      for(int8_t i=0;i<number_beep;i++) {
        WRITE(BEEPER,HIGH);
        delay(100);
        WRITE(BEEPER,LOW);
        delay(100);
      }
    #else
      for(int8_t i=0;i<number_beep;i++) {
        WRITE(BEEPER,HIGH);
        delay(1000000 / LCD_FEEDBACK_FREQUENCY_HZ / 2);
        WRITE(BEEPER,LOW);
        delay(1000000 / LCD_FEEDBACK_FREQUENCY_HZ / 2);
      }
    #endif
  #endif
}

inline void wait_heater() {
  setWatch();

  unsigned long timetemp = millis();

  /* See if we are heating up or cooling down */
  target_direction = isHeatingHotend(target_extruder); // true if heating, false if cooling

  cancel_heatup = false;

  #ifdef TEMP_RESIDENCY_TIME
    long residencyStart = -1;
    /* continue to loop until we have reached the target temp
      _and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
    while((!cancel_heatup)&&((residencyStart == -1) ||
          (residencyStart >= 0 && (((unsigned int) (millis() - residencyStart)) < (TEMP_RESIDENCY_TIME * 1000UL)))) )
  #else
    while ( target_direction ? (isHeatingHotend(target_extruder)) : (isCoolingHotend(target_extruder)&&(CooldownNoWait==false)) )
  #endif //TEMP_RESIDENCY_TIME

    { // while loop
      if (millis() > timetemp + 1000UL) { //Print temp & remaining time every 1s while waiting
        SERIAL_PROTOCOLPGM("T:");
        SERIAL_PROTOCOL_F(degHotend(target_extruder),1);
        SERIAL_PROTOCOLPGM(" E:");
        SERIAL_PROTOCOL((int)target_extruder);
        #ifdef TEMP_RESIDENCY_TIME
          SERIAL_PROTOCOLPGM(" W:");
          if (residencyStart > -1) {
            timetemp = ((TEMP_RESIDENCY_TIME * 1000UL) - (millis() - residencyStart)) / 1000UL;
            SERIAL_PROTOCOLLN( timetemp );
          }
          else {
            SERIAL_PROTOCOLLN( "?" );
          }
        #else
          SERIAL_PROTOCOLLN("");
        #endif
        timetemp = millis();
      }
      manage_heater();
      manage_inactivity();
      lcd_update();
      #ifdef TEMP_RESIDENCY_TIME
        // start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
        // or when current temp falls outside the hysteresis after target temp was reached
        if ((residencyStart == -1 &&  target_direction && (degHotend(target_extruder) >= (degTargetHotend(target_extruder)-TEMP_WINDOW))) ||
            (residencyStart == -1 && !target_direction && (degHotend(target_extruder) <= (degTargetHotend(target_extruder)+TEMP_WINDOW))) ||
            (residencyStart > -1 && labs(degHotend(target_extruder) - degTargetHotend(target_extruder)) > TEMP_HYSTERESIS) )
        {
          residencyStart = millis();
        }
      #endif //TEMP_RESIDENCY_TIME
    }

  LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
  refresh_cmd_timeout();
  starttime = previous_millis_cmd;
}

inline void wait_bed() {
  unsigned long timetemp = millis();

  cancel_heatup = false;
  target_direction = isHeatingBed(); // true if heating, false if cooling

  while ((target_direction)&&(!cancel_heatup) ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false)) ) {
    unsigned long ms = millis();
    if (ms > timetemp + 1000UL) { //Print Temp Reading every 1 second while heating up.
      timetemp = ms;
      float tt = degHotend(active_extruder);
      SERIAL_PROTOCOLPGM("T:");
      SERIAL_PROTOCOL(tt);
      SERIAL_PROTOCOLPGM(" E:");
      SERIAL_PROTOCOL((int)active_extruder);
      SERIAL_PROTOCOLPGM(" B:");
      SERIAL_PROTOCOL_F(degBed(), 1);
      SERIAL_PROTOCOLLN("");
    }
    manage_heater();
    manage_inactivity();
    lcd_update();
  }
  LCD_MESSAGEPGM(MSG_BED_DONE);
  refresh_cmd_timeout();
}

/******************************************************************************
***************************** G-Code Functions ********************************
*******************************************************************************/


/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1() {
  if (!Stopped) {

#if 0 //aven_0813
    #ifdef IDLE_OOZING_PREVENT
      IDLE_OOZING_retract(false);
    #endif
#endif

    get_coordinates(); // For X Y Z E F

#if 0 //aven_0813
    #ifdef FWRETRACT
      if (autoretract_enabled) {
        if (!(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
          float echange = destination[E_AXIS] - current_position[E_AXIS];
          // Is this move an attempt to retract or recover?
          if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
            current_position[E_AXIS] = destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations
            plan_set_e_position(current_position[E_AXIS]);  // AND from the planner
            retract(!retracted);
            return;
          }
        }
      }
    #endif //FWRETRACT
#endif

    prepare_move();
    //ClearToSend();
  }
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 */
inline void gcode_G2_G3(bool clockwise) {
  if (!Stopped) {
    get_arc_coordinates();
    prepare_arc_move(clockwise);
  }
}

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  unsigned long codenum = 0;

  LCD_MESSAGEPGM(MSG_DWELL);

  if (code_seen('P')) codenum = code_value_long(); // milliseconds to wait
  if (code_seen('S')) codenum = code_value_long() * 1000; // seconds to wait

  st_synchronize();
  refresh_cmd_timeout();
  codenum += previous_millis_cmd;  // keep track of when we started waiting
  while (millis() < codenum) {
    manage_heater();
    manage_inactivity();
    lcd_update();
  }
}

#ifdef FWRETRACT

  /**
   * G10 - Retract filament according to settings of M207
   * G11 - Recover filament according to settings of M208
   */
  inline void gcode_G10_G11(bool doRetract = false) {
    #if EXTRUDERS > 1
      if (doRetract) {
        retracted_swap[active_extruder] = (code_seen('S') && code_value_short() == 1); // checks for swap retract argument
      }
    #endif
    retract(doRetract
     #if EXTRUDERS > 1
      , retracted_swap[active_extruder]
     #endif
    );
  }

#endif //FWRETRACT

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 * If numbers are included with XYZ set the position as with G92
 * Currently adds the home_offset, which may be wrong and removed soon.
 *
 *  Xn  Home X, setting X to n + home_offset[X_AXIS]
 *  Yn  Home Y, setting Y to n + home_offset[Y_AXIS]
 *  Zn  Home Z, setting Z to n + home_offset[Z_AXIS]
 */
inline void gcode_G28(boolean home_x = false, boolean home_y = false)
{

  G28_f = 1;//aven_0807
  
  #ifdef ENABLE_AUTO_BED_LEVELING
    plan_bed_level_matrix.set_to_identity();
  #endif //ENABLE_AUTO_BED_LEVELING

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  refresh_cmd_timeout();

  enable_endstops(true);

  set_destination_to_current();

  feedrate = 0.0;

  bool  homeX = code_seen(axis_codes[X_AXIS]),
        homeY = code_seen(axis_codes[Y_AXIS]),
        homeZ = code_seen(axis_codes[Z_AXIS]),
        homeE = code_seen(axis_codes[E_AXIS]);
        
  home_all_axis = !(homeX || homeY || homeZ || homeE || home_x || home_y) || (homeX && homeY && homeZ);

#if 0
  #ifdef NPR2
    if((home_all_axis) || (code_seen(axis_codes[E_AXIS]))) {
      active_driver = active_extruder = 1;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], -200, COLOR_HOMERATE, active_extruder, active_driver);
      st_synchronize();
      old_color = 99;
      active_driver = active_extruder = 0;
    }
  #endif
#endif

  #ifdef DELTA
    // A delta can only safely home all axis at the same time
    // all axis have to home at the same time

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // Destination reached
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = destination[i];
    // take care of back off and rehome now we are all at the top
    
    HOMEAXIS(X);
	
    HOMEAXIS(Y);
	
    HOMEAXIS(Z);

    sync_plan_position_delta();

  #endif //DELTA

#if 0 //aven_0807
  #if defined(CARTESIAN) || defined(COREXY) || defined(SCARA)

    #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if (home_all_axis || homeZ) HOMEAXIS(Z);
    #elif !defined(Z_SAFE_HOMING) && defined(Z_RAISE_BEFORE_HOMING) && Z_RAISE_BEFORE_HOMING > 0
      // Raise Z before homing any other axes
      if (home_all_axis || homeZ) {
        destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
        feedrate = max_feedrate[Z_AXIS] * 60;
        line_to_destination();
        st_synchronize();
      }
    #endif

    #ifdef QUICK_HOME
      if (home_all_axis || (homeX && homeY)) {  // First diagonal move

        current_position[X_AXIS] = current_position[Y_AXIS] = 0;

        #ifdef DUAL_X_CARRIAGE
          int x_axis_home_dir = x_home_dir(active_extruder);
          extruder_duplication_enabled = false;
        #else
          int x_axis_home_dir = home_dir(X_AXIS);
        #endif

        sync_plan_position();

        float mlx = max_length(X_AXIS), mly = max_length(Y_AXIS),
              mlratio = mlx>mly ? mly/mlx : mlx/mly;

        destination[X_AXIS] = 1.5 * mlx * x_axis_home_dir;
        destination[Y_AXIS] = 1.5 * mly * home_dir(Y_AXIS);
        feedrate = min(homing_feedrate[X_AXIS], homing_feedrate[Y_AXIS]) * sqrt(mlratio * mlratio + 1);
        line_to_destination();
        st_synchronize();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        sync_plan_position();

        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        line_to_destination();
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose(); // clear endstop hit flags

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        #ifndef SCARA
          current_position[Z_AXIS] = destination[Z_AXIS];
        #endif
      }
    #endif // QUICK_HOME

    if (home_all_axis || homeX) {
      #ifdef DUAL_X_CARRIAGE
        int tmp_extruder = active_extruder;
        extruder_duplication_enabled = false;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_extruder_x_pos = current_position[X_AXIS];
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif //DUAL_X_CARRIAGE
    }

    // Home Y
    if (home_all_axis || homeY) HOMEAXIS(Y);

    // Set the X position, if included
    // Adds the home_offset as well, which may be wrong
    if (homeX) {
      float v = code_value();
      if (v) current_position[X_AXIS] = v
        #ifndef SCARA
          + home_offset[X_AXIS]
        #endif
      ;
    }

    // Set the Y position, if included
    // Adds the home_offset as well, which may be wrong
    if (homeY) {
      float v = code_value();
      if (v) current_position[Y_AXIS] = v
        #ifndef SCARA
          + home_offset[Y_AXIS]
        #endif
      ;
    }

    #if Z_HOME_DIR < 0  // If homing towards BED do Z last
      #ifndef Z_SAFE_HOMING
        if (code_seen('M') && !(homeX || homeY)) {
          // Manual G28 bed level
          #ifdef ULTIPANEL
            SERIAL_ECHOLN(" --LEVEL PLATE SCRIPT--");
            set_ChangeScreen(true);
            while(!lcd_clicked()) {
              set_pageShowInfo(0);
              lcd_update();
            }
            saved_feedrate = feedrate;
            saved_feedmultiply = feedmultiply;
            feedmultiply = 100;
            previous_millis_cmd = millis();

            enable_endstops(true);
            for(int8_t i=0; i < NUM_AXIS; i++) {
              destination[i] = current_position[i];
            }
            feedrate = 0.0;
            #if Z_HOME_DIR > 0  // If homing away from BED do Z first
              HOMEAXIS(Z);
            #endif
            HOMEAXIS(X);
            HOMEAXIS(Y);
            #if Z_HOME_DIR < 0
              HOMEAXIS(Z);
            #endif
            sync_plan_position();

            #ifdef ENDSTOPS_ONLY_FOR_HOMING
              enable_endstops(false);
            #endif

            feedrate = saved_feedrate;
            feedmultiply = saved_feedmultiply;
            previous_millis_cmd = millis();
            endstops_hit_on_purpose(); // clear endstop hit flags

            sync_plan_position();

            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS + 5);

            // PROBE FIRST POINT
            set_pageShowInfo(1);
            set_ChangeScreen(true);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {          
              manage_heater();
              manage_inactivity();
            }

            // PROBE SECOND POINT
            set_ChangeScreen(true);
            set_pageShowInfo(2);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // PROBE THIRD POINT
            set_ChangeScreen(true);
            set_pageShowInfo(3);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }     

            // PROBE FOURTH POINT
            set_ChangeScreen(true);
            set_pageShowInfo(4);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // PROBE CENTER
            set_ChangeScreen(true);
            set_pageShowInfo(5);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to((X_MAX_POS-X_MIN_POS)/2, (Y_MAX_POS-Y_MIN_POS)/2, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // FINISH MANUAL BED LEVEL
            set_ChangeScreen(true);
            set_pageShowInfo(6);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            enquecommands_P(PSTR("G28 X0 Y0\nG4 P0\nG4 P0\nG4 P0"));
          #endif // ULTIPANEL
        }
        else if(home_all_axis || homeZ) HOMEAXIS(Z);
      #elif defined(Z_SAFE_HOMING) && defined(ENABLE_AUTO_BED_LEVELING)// Z Safe mode activated.
        if (home_all_axis) {
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
          destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
          feedrate = xy_travel_speed;
          current_position[Z_AXIS] = 0;

          sync_plan_position();
          line_to_destination();
          st_synchronize();
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];
          HOMEAXIS(Z);
        }
        // Let's see if X and Y are homed and probe is inside bed area.
        if (homeZ) {
          if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) {
            float cpx = current_position[X_AXIS], cpy = current_position[Y_AXIS];
            if (   cpx >= X_MIN_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                && cpx <= X_MAX_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                && cpy >= Y_MIN_POS - Y_PROBE_OFFSET_FROM_EXTRUDER
                && cpy <= Y_MAX_POS - Y_PROBE_OFFSET_FROM_EXTRUDER) {
              current_position[Z_AXIS] = 0;
              plan_set_position(cpx, cpy, 0, current_position[E_AXIS]);
              destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
              feedrate = max_feedrate[Z_AXIS] * 60;
              line_to_destination();
              st_synchronize();
              HOMEAXIS(Z);
            }
            else {
              LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
              SERIAL_ECHO_START;
              SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
            }
          }
          else {
            LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
            SERIAL_ECHO_START;
            SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
          }
        }
      #elif defined(Z_SAFE_HOMING)
        if(home_all_axis || homeZ) {
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT);
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT);
          feedrate = xy_travel_speed;
          destination[Z_AXIS] = current_position[Z_AXIS] = 0;
          sync_plan_position();
          line_to_destination();
          st_synchronize();
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];

          HOMEAXIS(Z);
        }
      #endif //Z_SAFE_HOMING
    #endif //Z_HOME_DIR < 0

    // Set the Z position, if included
    // Adds the home_offset as well, which may be wrong
    if (homeZ) {
      float v = code_value();
      if (v) current_position[Z_AXIS] = v + home_offset[Z_AXIS];
    }

    #ifdef ENABLE_AUTO_BED_LEVELING && (Z_HOME_DIR < 0)
      if (home_all_axis || homeZ) current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
    #endif //ENABLE_AUTO_BED_LEVELING
    sync_plan_position();
  #endif // defined(CARTESIAN) || defined(COREXY) || defined(SCARA)

  #ifdef SCARA
    sync_plan_position_delta();
  #endif //SCARA
#endif //aven_0807


  #ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
  #endif

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  refresh_cmd_timeout();
  endstops_hit_on_purpose(); // clear endstop hit flags
  G28_f = 0; //aven_0807
}

#ifdef ENABLE_AUTO_BED_LEVELING

  /**
   * G29: Detailed Z-Probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *
   * Enhanced G29 Auto Bed Leveling Probe Routine
   * 
   * Parameters With AUTO_BED_LEVELING_GRID:
   *
   *  P  Set the size of the grid that will be probed (P x P points).
   *     Example: "G29 P4"
   *
   *  S  Set the XY travel speed between probe points (in mm/min)
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
   *     or clean the rotation Matrix. Useful to check the topology
   *     after a first run of G29.
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
   *     This is useful for manual bed leveling and finding flaws in the bed (to
   *     assist with part placement).
   *
   *  F  Set the Front limit of the probing grid
   *  B  Set the Back limit of the probing grid
   *  L  Set the Left limit of the probing grid
   *  R  Set the Right limit of the probing grid
   *
   * Global Parameters:
   *
   * E/e By default G29 will engages the probe, test the bed, then disengage.
   *     Include "E" to engage/disengage the probe for each sample.
   *     There's no extra effect if you have a fixed probe.
   *     Usage: "G29 E" or "G29 e"
   *
   */
  inline void gcode_G29() {

    // Don't allow auto-leveling without homing first
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
      return;
    }

    int verbose_level = code_seen('V') || code_seen('v') ? code_value_short() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      SERIAL_ECHOLNPGM("?(V)erbose Level is implausible (0-4).");
      return;
    }

    bool dryrun = code_seen('D') || code_seen('d'),
         deploy_probe_for_each_reading = code_seen('E') || code_seen('e');

    #ifdef AUTO_BED_LEVELING_GRID

      bool do_topography_map = verbose_level > 2 || code_seen('T') || code_seen('t');

      if (verbose_level > 0) {
        SERIAL_PROTOCOLPGM("G29 Auto Bed Leveling\n");
        if (dryrun) SERIAL_ECHOLNPGM("Running in DRY-RUN mode");
      }

      int auto_bed_leveling_grid_points = code_seen('P') ? code_value_long() : AUTO_BED_LEVELING_GRID_POINTS;
      if (auto_bed_leveling_grid_points < 2) {
        SERIAL_PROTOCOLPGM("?Number of probed (P)oints is implausible (2 minimum).\n");
        return;
      }

      xy_travel_speed = code_seen('S') ? code_value_short() : XY_TRAVEL_SPEED;

      int left_probe_bed_position = code_seen('L') ? code_value_short() : LEFT_PROBE_BED_POSITION,
          right_probe_bed_position = code_seen('R') ? code_value_short() : RIGHT_PROBE_BED_POSITION,
          front_probe_bed_position = code_seen('F') ? code_value_short() : FRONT_PROBE_BED_POSITION,
          back_probe_bed_position = code_seen('B') ? code_value_short() : BACK_PROBE_BED_POSITION;

      bool left_out_l = left_probe_bed_position < MIN_PROBE_X,
           left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - MIN_PROBE_EDGE,
           right_out_r = right_probe_bed_position > MAX_PROBE_X,
           right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
           front_out_f = front_probe_bed_position < MIN_PROBE_Y,
           front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - MIN_PROBE_EDGE,
           back_out_b = back_probe_bed_position > MAX_PROBE_Y,
           back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

      if (left_out || right_out || front_out || back_out) {
        if (left_out) {
          SERIAL_PROTOCOLPGM("?Probe (L)eft position out of range.\n");
          left_probe_bed_position = left_out_l ? MIN_PROBE_X : right_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (right_out) {
          SERIAL_PROTOCOLPGM("?Probe (R)ight position out of range.\n");
          right_probe_bed_position = right_out_r ? MAX_PROBE_X : left_probe_bed_position + MIN_PROBE_EDGE;
        }
        if (front_out) {
          SERIAL_PROTOCOLPGM("?Probe (F)ront position out of range.\n");
          front_probe_bed_position = front_out_f ? MIN_PROBE_Y : back_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (back_out) {
          SERIAL_PROTOCOLPGM("?Probe (B)ack position out of range.\n");
          back_probe_bed_position = back_out_b ? MAX_PROBE_Y : front_probe_bed_position + MIN_PROBE_EDGE;
        }
        return;
      }

    #endif // AUTO_BED_LEVELING_GRID

    #ifdef Z_PROBE_SLED
      dock_sled(false); // engage (un-dock) the probe
    #endif

    st_synchronize();

    if (!dryrun) {
      // make sure the bed_level_rotation_matrix is identity or the planner will get it wrong
      plan_bed_level_matrix.set_to_identity();

      //vector_3 corrected_position = plan_get_position_mm();
      //corrected_position.debug("position before G29");
      vector_3 uncorrected_position = plan_get_position();
      //uncorrected_position.debug("position during G29");

      current_position[X_AXIS] = uncorrected_position.x;
      current_position[Y_AXIS] = uncorrected_position.y;
      current_position[Z_AXIS] = uncorrected_position.z;
      sync_plan_position();
    }

    setup_for_endstop_move();
    feedrate = homing_feedrate[Z_AXIS];

    #ifdef AUTO_BED_LEVELING_GRID

      // probe at the points of a lattice grid
      const int xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (auto_bed_leveling_grid_points - 1),
                yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (auto_bed_leveling_grid_points - 1);

      // solve the plane equation ax + by + d = z
      // A is the matrix with rows [x y 1] for all the probed points
      // B is the vector of the Z positions
      // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
      // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

      int abl2 = auto_bed_leveling_grid_points * auto_bed_leveling_grid_points;

      double eqnAMatrix[abl2 * 3], // "A" matrix of the linear system of equations
             eqnBVector[abl2],     // "B" vector of Z points
             mean = 0.0;

      int probePointCounter = 0;
      bool zig = true;

      for (int yCount = 0; yCount < auto_bed_leveling_grid_points; yCount++) {
        double yProbe = front_probe_bed_position + yGridSpacing * yCount;
        int xStart, xStop, xInc;

        if (zig) {
          xStart = 0;
          xStop = auto_bed_leveling_grid_points;
          xInc = 1;
        }
        else {
          xStart = auto_bed_leveling_grid_points - 1;
          xStop = -1;
          xInc = -1;
        }


        // If topo_flag is set then don't zig-zag. Just scan in one direction.
        // This gets the probe points in more readable order.
        if (!do_topography_map) zig = !zig;
        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
          double xProbe = left_probe_bed_position + xGridSpacing * xCount;

          // raise extruder
          float measured_z,
                z_before = probePointCounter ? Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS] : Z_RAISE_BEFORE_PROBING;

          // Enhanced G29 - Do not retract servo between probes
          ProbeAction act;
          if (deploy_probe_for_each_reading) // G29 E - Stow between probes
            act = ProbeDeployAndStow;
          else if (yCount == 0 && xCount == 0)
            act = ProbeDeploy;
          else if (yCount == auto_bed_leveling_grid_points - 1 && xCount == auto_bed_leveling_grid_points - 1)
            act = ProbeStow;
          else
            act = ProbeStay;

          measured_z = probe_pt(xProbe, yProbe, z_before, act, verbose_level);

          mean += measured_z;

          eqnBVector[probePointCounter] = measured_z;
          eqnAMatrix[probePointCounter + 0 * abl2] = xProbe;
          eqnAMatrix[probePointCounter + 1 * abl2] = yProbe;
          eqnAMatrix[probePointCounter + 2 * abl2] = 1;

          probePointCounter++;

          manage_heater();
          manage_inactivity();
          lcd_update();

        } //xProbe
      } //yProbe

      clean_up_after_endstop_move();

      // solve lsq problem
      double *plane_equation_coefficients = qr_solve(abl2, 3, eqnAMatrix, eqnBVector);

      mean /= abl2;

      if (verbose_level) {
        SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[0], 8);
        SERIAL_PROTOCOLPGM(" b: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[1], 8);
        SERIAL_PROTOCOLPGM(" d: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[2], 8);
        SERIAL_EOL;
        if (verbose_level > 2) {
          SERIAL_PROTOCOLPGM("Mean of sampled points: ");
          SERIAL_PROTOCOL_F(mean, 8);
          SERIAL_EOL;
        }
      }

      if (do_topography_map) {

        SERIAL_PROTOCOLPGM(" \nBed Height Topography: \n");
        SERIAL_PROTOCOLPGM("+-----------+\n");
        SERIAL_PROTOCOLPGM("|...Back....|\n");
        SERIAL_PROTOCOLPGM("|Left..Right|\n");
        SERIAL_PROTOCOLPGM("|...Front...|\n");
        SERIAL_PROTOCOLPGM("+-----------+\n");

        for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
          for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
            int ind = yy * auto_bed_leveling_grid_points + xx;
            float diff = eqnBVector[ind] - mean;
            if (diff >= 0.0)
              SERIAL_PROTOCOLPGM(" +");   // Include + for column alignment
            else
              SERIAL_PROTOCOLCHAR(' ');
            SERIAL_PROTOCOL_F(diff, 5);
          } // xx
          SERIAL_EOL;
        } // yy
        SERIAL_EOL;

      } //do_topography_map

      if (!dryrun) set_bed_level_equation_lsq(plane_equation_coefficients);
      free(plane_equation_coefficients);

    #else // !AUTO_BED_LEVELING_GRID

      // Actions for each probe
      ProbeAction p1, p2, p3;
      if (deploy_probe_for_each_reading)
        p1 = p2 = p3 = ProbeDeployAndStow;
      else
        p1 = ProbeDeploy, p2 = ProbeStay, p3 = ProbeStow;

      // Probe at 3 arbitrary points
      float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING, p1, verbose_level),
            z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p2, verbose_level),
            z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p3, verbose_level);
      clean_up_after_endstop_move();
      if (!dryrun) set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);

    #endif // !AUTO_BED_LEVELING_GRID

    if (verbose_level > 0)
      plan_bed_level_matrix.debug(" \n\nBed Level Correction Matrix:");

    if (!dryrun) {
      // Correct the Z height difference from z-probe position and hotend tip position.
      // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
      // When the bed is uneven, this height must be corrected.
      float x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER,
            z_tmp = current_position[Z_AXIS],
            real_z = (float)st_get_position(Z_AXIS) / axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)

      apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
      current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
      sync_plan_position();
    }

    #ifdef Z_PROBE_SLED
      dock_sled(true, -SLED_DOCKING_OFFSET); // dock the probe, correcting for over-travel
    #endif
  }

  #ifndef Z_PROBE_SLED
    inline void gcode_G30() {
      deploy_z_probe(); // Engage Z Servo endstop if available
      st_synchronize();
      // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
      setup_for_endstop_move();

      feedrate = homing_feedrate[Z_AXIS];

      run_z_probe();
      SERIAL_PROTOCOLPGM(MSG_BED);
      SERIAL_PROTOCOLPGM(" X: ");
      SERIAL_PROTOCOL(current_position[X_AXIS] + 0.0001);
      SERIAL_PROTOCOLPGM(" Y: ");
      SERIAL_PROTOCOL(current_position[Y_AXIS] + 0.0001);
      SERIAL_PROTOCOLPGM(" Z: ");
      SERIAL_PROTOCOL(current_position[Z_AXIS] + 0.0001);
      SERIAL_EOL;

      clean_up_after_endstop_move();
      stow_z_probe(); // Retract Z Servo endstop if available
    }
  #endif //Z_PROBE_SLED
#endif //ENABLE_AUTO_BED_LEVELING

#ifdef DELTA
  // G29: Delta Z-Probe, probes the bed at more points.
  inline void gcode_G29() {
    if (code_seen('D')) {
      SERIAL_ECHOLN("Current bed level array values:");
      SERIAL_EOL;
      for (int y = 0; y < 7; y++) {
        for (int x = 0; x < 7; x++) {
          SERIAL_PROTOCOL_F(bed_level[x][y], 3);
          SERIAL_PROTOCOLPGM(" ");
        }
        SERIAL_EOL;
      }
      return;
    }
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;

    deploy_z_probe();
    calibrate_print_surface(z_probe_offset[Z_AXIS] + (code_seen(axis_codes[Z_AXIS]) ? code_value() : 0.0));
    retract_z_probe();

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    refresh_cmd_timeout();
    endstops_hit_on_purpose(); // clear endstop hit flags
  }

  // G30: Delta AutoCalibration
  inline void gcode_G30() {
    int iterations;
    //Zero the bed level array
    for (int y = 0; y < 7; y++) {
      for (int x = 0; x < 7; x++) {
        bed_level[x][y] = 0.0;
      }
    }

    if (code_seen('C')) {
      //Show carriage positions 
      SERIAL_ECHOLN("Carriage Positions for last scan:");
      for(int8_t i=0; i < 7; i++) {
        SERIAL_ECHO("[");
        SERIAL_ECHO(saved_positions[i][X_AXIS]);
        SERIAL_ECHO(", ");
        SERIAL_ECHO(saved_positions[i][Y_AXIS]);
        SERIAL_ECHO(", ");
        SERIAL_ECHO(saved_positions[i][Z_AXIS]);
        SERIAL_ECHOLN("]");
      }
      return;
    }

    if (code_seen('F')) {
      probing_feedrate=code_value();
    }

    if (code_seen('X') and code_seen('Y')) {
      //Probe specified X,Y point
      float x = code_seen('X') ? code_value():0.00;
      float y = code_seen('Y') ? code_value():0.00;
      float probe_value;

      deploy_z_probe();
      probe_value = probe_bed(x, y);
      SERIAL_ECHO("Bed Z-Height at X:");
      SERIAL_ECHO(x);
      SERIAL_ECHO(" Y:");
      SERIAL_ECHO(y);
      SERIAL_ECHO(" = ");
      SERIAL_PROTOCOL_F(probe_value, 4);
      SERIAL_EOL;

      SERIAL_ECHO("Carriage Positions: [");
      SERIAL_ECHO(saved_position[X_AXIS]);
      SERIAL_ECHO(", ");
      SERIAL_ECHO(saved_position[Y_AXIS]);
      SERIAL_ECHO(", ");
      SERIAL_ECHO(saved_position[Z_AXIS]);
      SERIAL_ECHOLN("]");
      retract_z_probe();
      return;
    }

    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;

    if (code_seen('A')) {
      SERIAL_ECHOLN("Starting Auto Calibration..");
      LCD_MESSAGEPGM("Auto Calibration...");
      if (code_value() != 0) ac_prec = code_value();
      SERIAL_ECHO("Calibration precision: +/-");
      SERIAL_PROTOCOL_F(ac_prec,3);
      SERIAL_ECHOLN("mm");

      //Zero the bedlevel array in case this affects bed probing
      for (int y = 0; y >=6; y++) {
        for (int x = 0; x >=6; y++) {
          bed_level[x][y] = 0.0;
        }
      }
    }

    home_delta_axis();
    deploy_z_probe(); 

    //Probe all points
    bed_probe_all();

    //Show calibration report      
    calibration_report();

    if (code_seen('A')) {
      iterations = 100; //Maximum number of iterations
      int loopcount = 1;
      float adj_r_target, adj_dr_target;
      float adj_r_target_delta = 0, adj_dr_target_delta = 0;
      float adj_AlphaA, adj_AlphaB, adj_AlphaC;
      float adj_RadiusA, adj_RadiusB, adj_RadiusC;
      float radiusErrorA, radiusErrorB,radiusErrorC;
      float adj_r = 0, adj_dr = 0;
      boolean equalAB, equalBC, equalCA;
      boolean adj_r_done, adj_dr_done, adj_tower_done;
      boolean adj_dr_allowed = true;
      float h_endstop = -100, l_endstop = 100;
      float probe_error, ftemp;  
      if (code_seen('D')) {
        delta_diagonal_rod = code_value();
        adj_dr_allowed = false;
        SERIAL_ECHOPAIR("Using diagional rod length: ", delta_diagonal_rod);
        SERIAL_ECHOLN("mm (will not be adjusted)");
      }

      //Check that endstop are within limits
      if (bed_level_x + endstop_adj[0] > h_endstop) h_endstop = bed_level_x + endstop_adj[0];
      if (bed_level_x + endstop_adj[0] < l_endstop) l_endstop = bed_level_x + endstop_adj[0];
      if (bed_level_y + endstop_adj[1] > h_endstop) h_endstop = bed_level_y + endstop_adj[1];
      if (bed_level_y + endstop_adj[1] < l_endstop) l_endstop = bed_level_y + endstop_adj[1];
      if (bed_level_z + endstop_adj[2] > h_endstop) h_endstop = bed_level_z + endstop_adj[2];
      if (bed_level_z + endstop_adj[2] < l_endstop) l_endstop = bed_level_z + endstop_adj[2];

      if (h_endstop - l_endstop > 3) {
        SERIAL_ECHOLN("The position of the endstop switches on this printer are not within limits");
        SERIAL_ECHOLN("Adjust endstop switches so that they are within 3mm Z-height of each other");
        SERIAL_EOL;
        SERIAL_ECHOPAIR("Current Endstop Positions - X: ", bed_level_x + endstop_adj[0]); 
        SERIAL_ECHOPAIR(" Y: ", bed_level_y + endstop_adj[1]);
        SERIAL_ECHOPAIR(" Z: ", bed_level_z + endstop_adj[2]);
        SERIAL_EOL;
        SERIAL_EOL;
        SERIAL_ECHOLN("Auto calibration aborted");

        retract_z_probe();

        //Restore saved variables
        feedrate = saved_feedrate;
        feedmultiply = saved_feedmultiply;
        return;
      }

      if (code_seen('D')) {
        //Fix diagonal rod at specified length (do not adjust)
        delta_diagonal_rod = code_value();
        adj_dr_allowed = false;
      }

      do {
        SERIAL_ECHO("Iteration: ");
        SERIAL_ECHO(loopcount);
        SERIAL_EOL;

        if ((bed_level_c > 3) or (bed_level_c < -3)) {
          //Build height is not set correctly .. 
          max_pos[Z_AXIS] -= bed_level_c + 2;
          set_delta_constants();
          SERIAL_ECHOPAIR("Adjusting Z-Height to: ", max_pos[Z_AXIS]);
          SERIAL_ECHOLN(" mm..");
        } 
        else {
          if ((bed_level_x < -ac_prec) or (bed_level_x > ac_prec) or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec) or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)) {
            //Endstop req adjustment
            SERIAL_ECHOLN("Adjusting Endstop..");
            endstop_adj[0] += bed_level_x / 1.05;
            endstop_adj[1] += bed_level_y / 1.05;
            endstop_adj[2] += bed_level_z / 1.05; 

            //Check that no endstop adj values are > 0 (not allowed).. if they are, reduce the build height to compensate.
            h_endstop = 0;
            for(int x=0; x < 3; x++) { 
              if (endstop_adj[x] > h_endstop) h_endstop = endstop_adj[x]; 
            }
            if (h_endstop > 0) {
              //Reduce build height and adjust endstop
              for(int x=0; x < 3; x++) {
                endstop_adj[x] -= h_endstop + 2;
              }
              max_pos[Z_AXIS] -= h_endstop + 2;
              set_delta_constants();
              SERIAL_ECHOPAIR("Adjusting Z-Height to: ", max_pos[Z_AXIS]);
              SERIAL_ECHOLN(" mm..");                
            }
          }
          else {
            SERIAL_ECHOLN("Endstop: OK");
            adj_r_target = (bed_level_x + bed_level_y + bed_level_z) / 3;
            adj_dr_target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;

            //Determine which parameters require adjustment
            if ((bed_level_c >= adj_r_target - ac_prec) and (bed_level_c <= adj_r_target + ac_prec)) adj_r_done = true; 
            else adj_r_done = false;
            if ((adj_dr_target >= adj_r_target - ac_prec) and (adj_dr_target <= adj_r_target + ac_prec)) adj_dr_done = true; 
            else adj_dr_done = false;
            if ((bed_level_x != bed_level_ox) or (bed_level_y != bed_level_oy) or (bed_level_z != bed_level_oz)) adj_tower_done = false; 
            else adj_tower_done = true;
            if ((adj_r_done == false) or (adj_dr_done == false) or (adj_tower_done == false)) {
              //delta geometry adjustment required
              SERIAL_ECHOLN("Adjusting Delta Geometry..");
              //set initial direction and magnitude for delta radius & diagonal rod adjustment
              if (adj_r == 0) {
                if (adj_r_target > bed_level_c) adj_r = 1; 
                else adj_r = -1;
              }

              if (adj_dr == 0) {
                if (adj_r_target > adj_dr_target) adj_dr = 1; 
                else adj_dr = -1;
              }

              //Don't adjust tower positions on first iteration
              adj_AlphaA = adj_AlphaB = adj_AlphaC = 0; 
              adj_RadiusA = adj_RadiusB = adj_RadiusC = 0;

              do {   
                //Apply adjustments 
                if (adj_r_done == false) {
                  SERIAL_ECHOPAIR("Adjusting Delta Radius (",delta_radius);
                  SERIAL_ECHOPAIR(" -> ", delta_radius + adj_r);
                  SERIAL_ECHOLN(")");
                  delta_radius += adj_r;
                }

                if (adj_dr_allowed == false) adj_dr_done = true;
                if (adj_dr_done == false) {
                  SERIAL_ECHOPAIR("Adjusting Diagonal Rod Length (",delta_diagonal_rod);
                  SERIAL_ECHOPAIR(" -> ", delta_diagonal_rod + adj_dr);
                  SERIAL_ECHOLN(")");
                  delta_diagonal_rod += adj_dr;
                }

                tower_adj[0] -= adj_AlphaA;
                tower_adj[1] -= adj_AlphaB;
                tower_adj[2] -= adj_AlphaC;
                tower_adj[3] += adj_RadiusA;
                tower_adj[4] += adj_RadiusB;
                tower_adj[5] += adj_RadiusC;

                set_delta_constants();

                bed_probe_all();
                calibration_report();

                //Check to see if auto calc is complete to within limits..
                if (adj_dr_allowed == true) {
                  if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
                    and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
                    and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
                    and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)
                    and (bed_level_ox >= -ac_prec) and (bed_level_ox <= ac_prec)
                    and (bed_level_oy >= -ac_prec) and (bed_level_oy <= ac_prec)
                    and (bed_level_oz >= -ac_prec) and (bed_level_oz <= ac_prec)) loopcount = iterations;
                } 
                else {
                  if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
                    and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
                    and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
                    and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) loopcount = iterations;
                }

                //set delta radius and diagonal rod targets
                adj_r_target = (bed_level_x + bed_level_y + bed_level_z) / 3;
                adj_dr_target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;

                //set Tower position adjustment values
                adj_AlphaA = bed_level_oy - bed_level_oz;
                adj_AlphaB = bed_level_oz - bed_level_ox;
                adj_AlphaC = bed_level_ox - bed_level_oy;

                //set tower radius errors
                radiusErrorA = bed_level_x - bed_level_ox;
                radiusErrorB = bed_level_y - bed_level_oy;
                radiusErrorC = bed_level_z - bed_level_oz;

                if ((radiusErrorA >= (radiusErrorB - 0.02)) and (radiusErrorA <= (radiusErrorB + 0.02))) equalAB = true;
                else equalAB = false;
                if ((radiusErrorB >= (radiusErrorC - 0.02)) and (radiusErrorB <= (radiusErrorC + 0.02))) equalBC = true;
                else equalBC = false;
                if ((radiusErrorC >= (radiusErrorA - 0.02)) and (radiusErrorC <= (radiusErrorA + 0.02))) equalCA = true;
                else equalCA = false;

                #ifdef DEBUG_MESSAGES
                  if (equalAB == true) {
                    SERIAL_ECHOPAIR("Tower AB Equal (A=",radiusErrorA);
                    SERIAL_ECHOPAIR(" B=",radiusErrorB);
                    SERIAL_ECHOLN(")");
                  }
                  else SERIAL_ECHOLN("equalAB=false");

                  if (equalBC == true) {
                    SERIAL_ECHOPAIR("Tower BC Equal (B=",radiusErrorB);
                    SERIAL_ECHOPAIR(" C=",radiusErrorC);
                    SERIAL_ECHOLN(")");
                  }
                  else SERIAL_ECHOLN("equalBC=false");

                  if (equalCA == true) {
                    SERIAL_ECHOPAIR("Tower CA Equal (C=",radiusErrorC);
                    SERIAL_ECHOPAIR(" A=",radiusErrorA);
                    SERIAL_ECHOLN(")");
                  }
                  else SERIAL_ECHOLN("equalCA=false");
                #endif //DEBUG_MESSAGES

                if ((equalAB == true) and (equalBC == true) and (equalCA == true)) {
                  // all tower radius out by the same amount (within 0.02) - allow adjustment with delta rod length
                  #ifdef DEBUG_MESSAGES
                    SERIAL_ECHOLN("All tower radius errors equal");
                  #endif
                  adj_RadiusA = adj_RadiusB = adj_RadiusC = 0;
                }

                if ((equalAB == true) and (equalBC == false) and (equalCA == false)) {
                  //Tower C radius error.. adjust it
                  SERIAL_ECHOLN("TowerC Radius error - adjusting");
                  if (adj_RadiusC == 0) {
                    if (bed_level_z < bed_level_oz) adj_RadiusC = 0.5;
                    if (bed_level_z > bed_level_oz) adj_RadiusC = -0.5;
                    #ifdef DEBUG_MESSAGES
                      SERIAL_ECHOPAIR("adj_RadiusC set to ",adj_RadiusC);
                      SERIAL_EOL;
                    #endif
                  }
                }

                if ((equalBC == true) and (equalAB == false) and (equalCA == false)) {
                  //Tower A radius error .. adjust it
                  SERIAL_ECHOLN("TowerA Radius error - adjusting");
                  if (adj_RadiusA == 0) {
                    if (bed_level_x < bed_level_ox) adj_RadiusA = 0.5;
                    if (bed_level_x > bed_level_ox) adj_RadiusA = -0.5;  
                    #ifdef DEBUG_MESSAGES
                      SERIAL_ECHOPAIR("adj_RadiusA set to ",adj_RadiusA);
                      SERIAL_EOL;
                    #endif
                  }
                } 

                if ((equalCA == true) and (equalAB == false) and (equalBC == false)) {
                  //Tower B radius error .. adjust it
                  SERIAL_ECHOLN("TowerB Radius error - adjusting");
                  if (adj_RadiusB == 0) {
                    if (bed_level_y < bed_level_oy) adj_RadiusB = 0.5;
                    if (bed_level_y > bed_level_oy) adj_RadiusB = -0.5;                     
                    #ifdef DEBUG_MESSAGES
                      SERIAL_ECHOPAIR("adj_RadiusB set to ",adj_RadiusB);
                      SERIAL_EOL;
                    #endif
                  }
                }                   

                if (((adj_r > 0) and (bed_level_c > adj_r_target)) or ((adj_r < 0) and (bed_level_c < adj_r_target))) {
                  //overshot target .. reverse & scale down
                  adj_r = -(adj_r / 2);
                }

                if (((adj_dr > 0) and (adj_dr_target > adj_r_target)) or ((adj_dr < 0) and (adj_dr_target < adj_r_target))) {
                  //overshot target .. reverse & scale down
                  adj_dr = -(adj_dr / 2);
                }

                //Tower radius overshot targets?
                if (((adj_RadiusA > 0) and (bed_level_x > bed_level_ox)) or ((adj_RadiusA < 0) and (bed_level_x < bed_level_ox))) adj_RadiusA = -(adj_RadiusA / 2);
                if (((adj_RadiusB > 0) and (bed_level_y > bed_level_oy)) or ((adj_RadiusB < 0) and (bed_level_y < bed_level_oy))) adj_RadiusB = -(adj_RadiusB / 2);
                if (((adj_RadiusC > 0) and (bed_level_z > bed_level_oz)) or ((adj_RadiusC < 0) and (bed_level_z < bed_level_oz))) adj_RadiusC = -(adj_RadiusC / 2);

                //Delta radius adjustment complete?                       
                if ((bed_level_c >= (adj_r_target - ac_prec)) and (bed_level_c <= (adj_r_target + ac_prec))) adj_r_done = true; 
                else adj_r_done = false;

                //Diag Rod adjustment complete?
                if ((adj_dr_target >= (adj_r_target - ac_prec)) and (adj_dr_target <= (adj_r_target + ac_prec))) adj_dr_done = true; 
                else adj_dr_done = false;

                #ifdef DEBUG_MESSAGES
                  SERIAL_ECHOPAIR("c: ", bed_level_c);
                  SERIAL_ECHOPAIR(" x: ", bed_level_x);
                  SERIAL_ECHOPAIR(" y: ", bed_level_y);
                  SERIAL_ECHOPAIR(" z: ", bed_level_z);
                  SERIAL_ECHOPAIR(" ox: ", bed_level_ox);
                  SERIAL_ECHOPAIR(" oy: ", bed_level_oy);
                  SERIAL_ECHOPAIR(" oz: ", bed_level_oz);
                  SERIAL_EOL;
                  SERIAL_ECHO("radius:");
                  SERIAL_PROTOCOL_F(delta_radius, 4);
                  SERIAL_ECHO(" diagrod:");
                  SERIAL_PROTOCOL_F(delta_diagonal_rod, 4);
                  SERIAL_EOL;
                  SERIAL_ECHO("Radius Adj Complete: ");
                  if (adj_r_done == true) SERIAL_ECHO("Yes"); 
                  else SERIAL_ECHO("No");
                  SERIAL_ECHO(" DiagRod Adj Complete: ");
                  if (adj_dr_done == true) SERIAL_ECHO("Yes"); 
                  else SERIAL_ECHO("No");
                  SERIAL_EOL;
                  SERIAL_ECHOPAIR("RadiusA Error: ",radiusErrorA);
                  SERIAL_ECHOPAIR(" (adjust: ",adj_RadiusA);
                  SERIAL_ECHOLN(")");
                  SERIAL_ECHOPAIR("RadiusB Error: ",radiusErrorB);
                  SERIAL_ECHOPAIR(" (adjust: ",adj_RadiusB);
                  SERIAL_ECHOLN(")");
                  SERIAL_ECHOPAIR("RadiusC Error: ",radiusErrorC);
                  SERIAL_ECHOPAIR(" (adjust: ",adj_RadiusC);
                  SERIAL_ECHOLN(")");
                  SERIAL_ECHOPAIR("DeltaAlphaA: ",adj_AlphaA);
                  SERIAL_EOL;
                  SERIAL_ECHOPAIR("DeltaAlphaB: ",adj_AlphaB);
                  SERIAL_EOL;
                  SERIAL_ECHOPAIR("DeltaAlphaC: ",adj_AlphaC);
                  SERIAL_EOL;
                #endif
              } while (((adj_r_done == false) or (adj_dr_done = false)) and (loopcount < iterations));
            }
            else {
              SERIAL_ECHOLN("Delta Geometry: OK");
            }
          }
        }

        if (loopcount < iterations) {

			//aven_0813
		  delay(500);	
          home_delta_axis();

          //probe bed and display report
          bed_probe_all();
          calibration_report();

          //Check to see if autocalc is complete to within limits..
          if (adj_dr_allowed == true) {
            if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
              and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
              and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
              and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)
              and (bed_level_ox >= -ac_prec) and (bed_level_ox <= ac_prec)
              and (bed_level_oy >= -ac_prec) and (bed_level_oy <= ac_prec)
              and (bed_level_oz >= -ac_prec) and (bed_level_oz <= ac_prec)) loopcount = iterations;
          }
          else {
            if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
              and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
              and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
              and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) loopcount = iterations;
          }
        }
        loopcount ++;
      } while(loopcount < iterations);

      SERIAL_ECHOLN("Auto Calibration Complete");
      LCD_MESSAGEPGM("Complete");
      SERIAL_ECHOLN("Issue M500 Command to save calibration settings to EPROM (if enabled)");
      /*   
       if ((abs(delta_diagonal_rod - saved_delta_diagonal_rod) > 1) and (adj_dr_allowed == true)) {
       SERIAL_EOL;
       SERIAL_ECHOPAIR("WARNING: The length of diagonal rods specified (", saved_delta_diagonal_rod);
       SERIAL_ECHOLN(" mm) appears to be incorrect");
       SERIAL_ECHOLN("If you have measured your rods and you believe that this value is correct, this could indicate");
       SERIAL_ECHOLN("excessive twisting movement of carriages and/or loose screws/joints on carriages or end effector");
       }
       */
    }

    retract_z_probe();

    //Restore saved variables
    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
  }
#endif // DELTA

// G60: Store in memory actual position
inline void gcode_G60() {
  lastpos[X_AXIS]=current_position[X_AXIS];
  lastpos[Y_AXIS]=current_position[Y_AXIS];
  lastpos[Z_AXIS]=current_position[Z_AXIS];
  lastpos[E_AXIS]=current_position[E_AXIS];
  //SERIAL_ECHOPAIR(" Lastpos X: ", lastpos[X_AXIS]);
  //SERIAL_ECHOPAIR(" Lastpos Y: ", lastpos[Y_AXIS]);
  //SERIAL_ECHOPAIR(" Lastpos Z: ", lastpos[Z_AXIS]);
  //SERIAL_ECHOPAIR(" Lastpos E: ", lastpos[E_AXIS]);
  //SERIAL_EOL;
}

// G61: move to X Y Z in memory
inline void gcode_G61() {
  for(int8_t i = 0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) {
      destination[i] = (float)code_value() + lastpos[i];
    }
    else {
      destination[i] = current_position[i];
    }
  }
  //SERIAL_ECHOPAIR(" Move to X: ", destination[X_AXIS]);
  //SERIAL_ECHOPAIR(" Move to Y: ", destination[Y_AXIS]);
  //SERIAL_ECHOPAIR(" Move to Z: ", destination[Z_AXIS]);
  //SERIAL_ECHOPAIR(" Move to E: ", destination[E_AXIS]);
  //SERIAL_EOL;

  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
  //finish moves
  prepare_move();
}

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  if (!code_seen(axis_codes[E_AXIS]))
    st_synchronize();

  bool didXYZ = false;
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      float v = current_position[i] = code_value();
      if (i == E_AXIS)
        plan_set_e_position(v);
      else
        didXYZ = true;
    }
  }
  if (didXYZ) sync_plan_position();
}

#ifdef ULTIPANEL

  /**
   * M0: // M0 - Unconditional stop - Wait for user button press on LCD
   * M1: // M1 - Conditional stop - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    char *src = strchr_pointer + 2;

    unsigned long codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_short(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value_short() * 1000UL; // seconds to wait
      hasS = codenum > 0;
    }
    char* starpos = strchr(src, '*');
    if (starpos != NULL) *(starpos) = '\0';
    while (*src == ' ') ++src;
    if (!hasP && !hasS && *src != '\0')
      lcd_setstatus(src, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if defined(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        dontExpireStatus();
      #endif
    }

    lcd_ignore_click();
    st_synchronize();
    refresh_cmd_timeout();
    if (codenum > 0) {
      codenum += previous_millis_cmd;  // keep track of when we started waiting
      while(millis() < codenum && !lcd_clicked()) {
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
      lcd_ignore_click(false);
    }
    else {
      if (!lcd_detected()) return;
      while (!lcd_clicked()) {
        manage_heater();
        manage_inactivity();
        lcd_update();
      }
    }
    if (IS_SD_PRINTING)
      LCD_MESSAGEPGM(MSG_RESUMING);
    else
      LCD_MESSAGEPGM(WELCOME_MSG);
  }
#endif //ULTIPANEL

#ifdef LASERBEAM
  // M3: S - Setting laser beam
  inline void gcode_M3() {
    if(code_seen('S')) {
      laser_ttl_modulation = constrain(code_value(),0,255);
    }
    else {
      laser_ttl_modulation=0;
    }
  }
  // M4: Turn on laser beam
  inline void gcode_M4() {
    WRITE(LASER_PWR_PIN, HIGH);
    laser_ttl_modulation = 0;
  }
  // M5: Turn off laser beam
  inline void gcode_M5() {
    WRITE(LASER_PWR_PIN, LOW);
    laser_ttl_modulation=0;
  }
#endif //LASERBEAM

#ifdef FILAMENT_END_SWITCH
  // M11: Start printing
  inline void gcode_M11() {
    printing = true;
    paused = false;
    SERIAL_ECHOLN("Start Printing, pause pin active.");
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }
#endif

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}

#ifdef SDSUPPORT

  /**
   * M20: List SD card to serial output
   */
  inline void gcode_M20() {
    SERIAL_PROTOCOLLNPGM(MSG_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_PROTOCOLLNPGM(MSG_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21() {
    card.initsd();
  }

  /**
   * M22: Release SD Card
   */
  inline void gcode_M22() {
    card.release();
  }

  /**
   * M23: Select a file
   */
  inline void gcode_M23() {
    char* codepos = strchr_pointer + 4;
    char* starpos = strchr(codepos, '*');
    if (starpos) *starpos = '\0';
    card.openFile(codepos, true);
  }

  /**
   * M24: Start SD Print
   */
  inline void gcode_M24() {
    card.startFileprint();
    starttime = millis();
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() {
    card.pauseSDPrint();
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26() {
    if (card.cardOK && code_seen('S'))
      card.setIndex(code_value_short());
  }

  /**
   * M27: Get SD Card status
   */
  inline void gcode_M27() {
    card.getStatus();
  }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28() {
    char* codepos = strchr_pointer + 4;
    char* starpos = strchr(codepos, '*');
    if (starpos) {
      char* npos = strchr(cmdbuffer[bufindr], 'N');
      strchr_pointer = strchr(npos, ' ') + 1;
      *(starpos) = '\0';
    }
    card.openFile(codepos, false);
  }

  /**
   * M29: Stop SD Write
   * Processed in write to file routine above
   */
  inline void gcode_M29() {
    // card.saving = false;
  }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30() {
    if (card.cardOK) {
      card.closefile();
      char* starpos = strchr(strchr_pointer + 4, '*');
      if (starpos) {
        char* npos = strchr(cmdbuffer[bufindr], 'N');
        strchr_pointer = strchr(npos, ' ') + 1;
        *(starpos) = '\0';
      }
      card.removeFile(strchr_pointer + 4);
    }
  }
#endif

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
  stoptime = millis();
  unsigned long t = (stoptime - starttime) / 1000;
  int min = t / 60, sec = t % 60;
  char time[30];
  sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
  SERIAL_ECHO_START;
  SERIAL_ECHOLN(time);
  lcd_setstatus(time);
  autotempShutdown();
}

#ifdef SDSUPPORT

  /**
   * M32: Select file and start SD Print
   */
  inline void gcode_M32() {
    if (card.sdprinting)
      st_synchronize();

    char* codepos = strchr_pointer + 4;

    char* namestartpos = strchr(codepos, '!');   //find ! to indicate filename string start.
    if (! namestartpos)
      namestartpos = codepos; //default name position, 4 letters after the M
    else
      namestartpos++; //to skip the '!'

    char* starpos = strchr(codepos, '*');
    if (starpos) *(starpos) = '\0';

    bool call_procedure = code_seen('P') && (strchr_pointer < namestartpos);

    if (card.cardOK) {
      card.openFile(namestartpos, true, !call_procedure);

      if (code_seen('S') && strchr_pointer < namestartpos) // "S" (must occur _before_ the filename!)
        card.setIndex(code_value_short());

      card.startFileprint();
      if (!call_procedure) {
        starttime = millis(); //procedure calls count as normal print time.
        #if HAS_POWER_CONSUMPTION_SENSOR
          startpower = power_consumption_hour;
        #endif
      }
    }
  }

  /**
   * M928: Start SD Write
   */
  inline void gcode_M928() {
    char* starpos = strchr(strchr_pointer + 5, '*');
    if (starpos) {
      char* npos = strchr(cmdbuffer[bufindr], 'N');
      strchr_pointer = strchr(npos, ' ') + 1;
      *(starpos) = '\0';
    }
    card.openLogFile(strchr_pointer + 5);
  }

#endif // SDSUPPORT

/**
 * M42: Change pin status via GCode
 */
inline void gcode_M42() {
  if (code_seen('S')) {
    int pin_status = code_value_short(),
        pin_number = LED_PIN;

    if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
      pin_number = code_value_short();

    for (int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins) / sizeof(*sensitive_pins)); i++) {
      if (sensitive_pins[i] == pin_number) {
        pin_number = -1;
        break;
      }
    }

    #if HAS_FAN
      if (pin_number == FAN_PIN) fanSpeed = pin_status;
    #endif

    if (pin_number > -1) {
      pinMode(pin_number, OUTPUT);
      digitalWrite(pin_number, pin_status);
      analogWrite(pin_number, pin_status);
    }
  } // code_seen('S')
}

#if defined(ENABLE_AUTO_BED_LEVELING) && defined(Z_PROBE_REPEATABILITY_TEST)
  /**
   * M49: Z-Probe repeatability measurement function.
   *
   * Usage:
   *   M49 <n#> <X#> <Y#> <V#> <E> <L#>
   *     n = Number of sampled points (4-50, default 10)
   *     X = Sample X position
   *     Y = Sample Y position
   *     V = Verbose level (0-4, default=1)
   *     E = Engage probe for each reading
   *     L = Number of legs of movement before probe
   *  
   * This function assumes the bed has been homed.  Specifically, that a G28 command
   * as been issued prior to invoking the M48 Z-Probe repeatability measurement function.
   * Any information generated by a prior G29 Bed leveling command will be lost and need to be
   * regenerated.
   *
   * The number of samples will default to 10 if not specified.  You can use upper or lower case
   * letters for any of the options EXCEPT n.  n must be in lower case because Marlin uses a capital
   * N for its communication protocol and will get horribly confused if you send it a capital N.
   */
  inline void gcode_M49() {

    double sum = 0.0, mean = 0.0, sigma = 0.0, sample_set[50];
    int verbose_level = 1, n_samples = 10, n_legs = 0;

    if (code_seen('V') || code_seen('v')) {
      verbose_level = code_value_short();
      if (verbose_level < 0 || verbose_level > 4 ) {
        SERIAL_PROTOCOLPGM("?Verbose Level not plausible (0-4).\n");
        return;
      }
    }

    if (verbose_level > 0)
      SERIAL_PROTOCOLPGM("M49 Z-Probe Repeatability test\n");

    if (code_seen('P') || code_seen('p') || code_seen('n')) { // `n` for legacy support only - please use `P`!
      n_samples = code_value_short();
      if (n_samples < 4 || n_samples > 50) {
        SERIAL_PROTOCOLPGM("?Sample size not plausible (4-50).\n");
        return;
      }
    }

    double X_probe_location, Y_probe_location,
           X_current = X_probe_location = st_get_position_mm(X_AXIS),
           Y_current = Y_probe_location = st_get_position_mm(Y_AXIS),
           Z_current = st_get_position_mm(Z_AXIS),
           Z_start_location = Z_current + Z_RAISE_BEFORE_PROBING,
           ext_position = st_get_position_mm(E_AXIS);

    bool deploy_probe_for_each_reading = code_seen('E') || code_seen('e');

    if (code_seen('X') || code_seen('x')) {
      X_probe_location = code_value() - X_PROBE_OFFSET_FROM_EXTRUDER;
      if (X_probe_location < X_MIN_POS || X_probe_location > X_MAX_POS) {
        SERIAL_PROTOCOLPGM("?X position out of range.\n");
        return;
      }
    }

    if (code_seen('Y') || code_seen('y')) {
      Y_probe_location = code_value() -  Y_PROBE_OFFSET_FROM_EXTRUDER;
      if (Y_probe_location < Y_MIN_POS || Y_probe_location > Y_MAX_POS) {
        SERIAL_PROTOCOLPGM("?Y position out of range.\n");
        return;
      }
    }

    if (code_seen('L') || code_seen('l')) {
      n_legs = code_value_short();
      if (n_legs == 1) n_legs = 2;
      if (n_legs < 0 || n_legs > 15) {
        SERIAL_PROTOCOLPGM("?Number of legs in movement not plausible (0-15).\n");
        return;
      }
    }

    //
    // Do all the preliminary setup work. First raise the probe.
    //

    st_synchronize();
    plan_bed_level_matrix.set_to_identity();
    plan_buffer_line(X_current, Y_current, Z_start_location, ext_position, homing_feedrate[Z_AXIS]/60, active_extruder, active_driver);
    st_synchronize();

    //
    // Now get everything to the specified probe point So we can safely do a probe to
    // get us close to the bed.  If the Z-Axis is far from the bed, we don't want to 
    // use that as a starting point for each probe.
    //
    if (verbose_level > 2)
      SERIAL_PROTOCOL("Positioning the probe...\n");

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, ext_position, homing_feedrate[X_AXIS]/60, active_extruder, active_driver);
    st_synchronize();

    current_position[X_AXIS] = X_current = st_get_position_mm(X_AXIS);
    current_position[Y_AXIS] = Y_current = st_get_position_mm(Y_AXIS);
    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
    current_position[E_AXIS] = ext_position = st_get_position_mm(E_AXIS);

    // 
    // OK, do the initial probe to get us close to the bed.
    // Then retrace the right amount and use that in subsequent probes
    //

    deploy_z_probe();

    setup_for_endstop_move();
    run_z_probe();

    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
    Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;

    plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, ext_position, homing_feedrate[X_AXIS]/60, active_extruder, active_driver);
    st_synchronize();
    current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);

    if (deploy_probe_for_each_reading) stow_z_probe();

    for (uint16_t n=0; n < n_samples; n++) {

      do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // Make sure we are at the probe location

      if (n_legs) {
        unsigned long ms = millis();
        double radius = ms % (X_MAX_LENGTH / 4),       // limit how far out to go
               theta = RADIANS(ms % 360L);
        float dir = (ms & 0x0001) ? 1 : -1;            // clockwise or counter clockwise

        //SERIAL_ECHOPAIR("starting radius: ",radius);
        //SERIAL_ECHOPAIR("   theta: ",theta);
        //SERIAL_ECHOPAIR("   direction: ",dir);
        //SERIAL_EOL;

        for (int l = 0; l < n_legs - 1; l++) {
          ms = millis();
          theta += RADIANS(dir * (ms % 20L));
          radius += (ms % 10L) - 5L;
          if (radius < 0.0) radius = -radius;

          X_current = X_probe_location + cos(theta) * radius;
          Y_current = Y_probe_location + sin(theta) * radius;
          X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
          Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);

          if (verbose_level > 3) {
            SERIAL_ECHOPAIR("x: ", X_current);
            SERIAL_ECHOPAIR("y: ", Y_current);
            SERIAL_EOL;
          }

          do_blocking_move_to(X_current, Y_current, Z_current);

        } // n_legs loop

        do_blocking_move_to(X_probe_location, Y_probe_location, Z_start_location); // Go back to the probe location

      } // n_legs

      if (deploy_probe_for_each_reading) {
        deploy_z_probe(); 
        delay(1000);
      }

      setup_for_endstop_move();
      run_z_probe();

      sample_set[n] = current_position[Z_AXIS];

      //
      // Get the current mean for the data points we have so far
      //
      sum = 0.0;
      for (int j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      //
      // Now, use that mean to calculate the standard deviation for the
      // data points we have so far
      //
      sum = 0.0;
      for (int j = 0; j <= n; j++) {
        float ss = sample_set[j] - mean;
        sum += ss * ss;
      }
      sigma = sqrt(sum / (n + 1));

      if (verbose_level > 1) {
        SERIAL_PROTOCOL(n+1);
        SERIAL_PROTOCOLPGM(" of ");
        SERIAL_PROTOCOL(n_samples);
        SERIAL_PROTOCOLPGM("   z: ");
        SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
        if (verbose_level > 2) {
          SERIAL_PROTOCOLPGM(" mean: ");
          SERIAL_PROTOCOL_F(mean,6);
          SERIAL_PROTOCOLPGM("   sigma: ");
          SERIAL_PROTOCOL_F(sigma,6);
        }
      }

      if (verbose_level > 0) SERIAL_EOL;

      plan_buffer_line(X_probe_location, Y_probe_location, Z_start_location, current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder, active_driver);
      st_synchronize();

      if (deploy_probe_for_each_reading) {
        stow_z_probe();
        delay(1000);
      }
    }

    if (!deploy_probe_for_each_reading) {
      stow_z_probe();
      delay(1000);
    }

    clean_up_after_endstop_move();

    // enable_endstops(true);

    if (verbose_level > 0) {
      SERIAL_PROTOCOLPGM("Mean: ");
      SERIAL_PROTOCOL_F(mean, 6);
      SERIAL_EOL;
    }

    SERIAL_PROTOCOLPGM("Standard Deviation: ");
    SERIAL_PROTOCOL_F(sigma, 6);
    SERIAL_EOL; SERIAL_EOL;
  }

#endif // ENABLE_AUTO_BED_LEVELING && Z_PROBE_REPEATABILITY_TEST

#if HAS_POWER_SWITCH

  /**
   * M80: Turn on Power Supply
   */
  inline void gcode_M80() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); //GND

    // If you have a switch on suicide pin, this is useful
    // if you want to start another print with suicide feature after
    // a print without suicide...
    #if HAS_SUICIDE
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #ifdef ULTIPANEL
      powersupply = true;
      LCD_MESSAGEPGM(WELCOME_MSG);
      lcd_update();
    #endif
  }
#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  disable_heater();
  st_synchronize();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
  finishAndDisableSteppers();
  fanSpeed = 0;
  delay(1000); // Wait 1 second before switching off
  #if HAS_SUICIDE
    st_synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
  #endif
  #ifdef ULTIPANEL
    #if HAS_POWER_SWITCH
      powersupply = false;
    #endif
    LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");
    lcd_update();
  #endif
}

/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable all stepper motors
 */
inline void gcode_M18_M84() {
  if (code_seen('S')) {
    stepper_inactive_time = code_value() * 1000;
  }
  else {
    bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
    if (all_axis) {
      st_synchronize();
      disable_e();
      finishAndDisableSteppers();
    }
    else {
      st_synchronize();
      if (code_seen('X')) disable_x();
      if (code_seen('Y')) disable_y();
      if (code_seen('Z')) disable_z();
      #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
        if (code_seen('E')) {
          disable_e();
        }
      #endif
    }
  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (code_seen('S')) max_inactive_time = code_value() * 1000;
}

/**
 * M92: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M92() {
  for(int8_t i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS) {
        float value = code_value();
        if (value < 20.0) {
          float factor = axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
          max_e_jerk *= factor;
          max_feedrate[i] *= factor;
          axis_steps_per_sqr_second[i] *= factor;
        }
        axis_steps_per_unit[i] = value;
      }
      else {
        axis_steps_per_unit[i] = code_value();
      }
    }
  }
}

/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
  if (setTargetedHotend(104)) return;
  if (debugDryrun()) return;
  #if HOTENDS == 1
    if (target_extruder != active_extruder) return;
  #endif
  if (code_seen('S')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #ifdef DUAL_X_CARRIAGE
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif
    setWatch();
  }
}

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
  if (setTargetedHotend(105)) return;
  if (debugDryrun()) return;
  #if HAS_TEMP_0 || HAS_TEMP_BED
    SERIAL_PROTOCOLPGM("ok");
    #if HAS_TEMP_0
      SERIAL_PROTOCOLPGM(" T:");
      SERIAL_PROTOCOL_F(degHotend(target_extruder), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetHotend(target_extruder), 1);
    #endif
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM(" B:");
      SERIAL_PROTOCOL_F(degBed(), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetBed(), 1);
    #endif
    for (int8_t e = 0; e < EXTRUDERS; ++e) {
      SERIAL_PROTOCOLPGM(" T");
      SERIAL_PROTOCOL(e);
      SERIAL_PROTOCOLCHAR(':');
      SERIAL_PROTOCOL_F(degHotend(e), 1);
      SERIAL_PROTOCOLPGM(" /");
      SERIAL_PROTOCOL_F(degTargetHotend(e), 1);
    }
  #else // !HAS_TEMP_0 && !HAS_TEMP_BED
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_PROTOCOLPGM(" @:");
  #ifdef HOTEND_WATTS
    SERIAL_PROTOCOL((HOTEND_WATTS * getHeaterPower(target_extruder))/127);
    SERIAL_PROTOCOLCHAR('W');
  #else
    SERIAL_PROTOCOL(getHeaterPower(target_extruder));
  #endif

  SERIAL_PROTOCOLPGM(" B@:");
  #ifdef BED_WATTS
    SERIAL_PROTOCOL((BED_WATTS * getHeaterPower(-1))/127);
    SERIAL_PROTOCOLCHAR('W');
  #else
    SERIAL_PROTOCOL(getHeaterPower(-1));
  #endif

  #ifdef SHOW_TEMP_ADC_VALUES
    #if HAS_TEMP_BED
      SERIAL_PROTOCOLPGM("    ADC B:");
      SERIAL_PROTOCOL_F(degBed(),1);
      SERIAL_PROTOCOLPGM("C->");
      SERIAL_PROTOCOL_F(rawBedTemp()/OVERSAMPLENR,0);
    #endif
    for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
      SERIAL_PROTOCOLPGM("  T");
      SERIAL_PROTOCOL(cur_extruder);
      SERIAL_PROTOCOLCHAR(':');
      SERIAL_PROTOCOL_F(degHotend(cur_extruder),1);
      SERIAL_PROTOCOLPGM("C->");
      SERIAL_PROTOCOL_F(rawHotendTemp(cur_extruder)/OVERSAMPLENR,0);
    }
  #endif

  SERIAL_EOL;
}

#if HAS_FAN
  /**
   * M106: Set Fan Speed
   */
  inline void gcode_M106() { fanSpeed = code_seen('S') ? constrain(code_value_short(), 0, 255) : 255; }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() { fanSpeed = 0; }

#endif // HAS_FAN

/**
 * M109: Wait for extruder(s) to reach temperature
 */
inline void gcode_M109() {
  if (setTargetedHotend(109)) return;
  if (debugDryrun()) return;
  #if HOTENDS == 1
    if (target_extruder != active_extruder) return;
  #endif
  LCD_MESSAGEPGM(MSG_HEATING);

  CooldownNoWait = code_seen('S');
  if (CooldownNoWait || code_seen('R')) {
    float temp = code_value();
    setTargetHotend(temp, target_extruder);
    #ifdef DUAL_X_CARRIAGE
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        setTargetHotend1(temp == 0.0 ? 0.0 : temp + duplicate_extruder_temp_offset);
    #endif
  }

  #ifdef AUTOTEMP
    autotemp_enabled = code_seen('F');
    if (autotemp_enabled) autotemp_factor = code_value();
    if (code_seen('S')) autotemp_min = code_value();
    if (code_seen('B')) autotemp_max = code_value();
  #endif

  wait_heater();
}

/**
 * M111: Debug mode Repetier Host compatibile
 */
inline void gcode_M111() {
  if (code_seen('S')) debugLevel = code_value_short();
  if (debugDryrun()) {
    SERIAL_ECHOLN("DEBUG DRYRUN ENABLED");
    setTargetBed(0);
    for (int8_t cur_hotend = 0; cur_hotend < HOTENDS; ++cur_hotend) {
      setTargetHotend(0, cur_hotend);
    }
  }
}

/**
 * M112: Emergency Stop
 */
inline void gcode_M112() {
  kill();
}

/**
 * M114: Output current position to serial port
 */
inline void gcode_M114() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(current_position[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(current_position[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(current_position[Z_AXIS]);
  SERIAL_PROTOCOLPGM(" E:");
  SERIAL_PROTOCOL(current_position[E_AXIS]);

  SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

  SERIAL_EOL;

  #if 0
  SerialUSB.print("X:");
  SerialUSB.print(st_get_position(X_AXIS));
  SerialUSB.print(" Y:");
  SerialUSB.print(st_get_position(Y_AXIS));
  SerialUSB.print(" Z:");
  SerialUSB.println(st_get_position(Z_AXIS));
  #endif

#if 0 //aven_0813
  #ifdef SCARA
    SERIAL_PROTOCOLPGM("SCARA Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta:");
    SERIAL_PROTOCOL(delta[Y_AXIS]);
    SERIAL_EOL;
    
    SERIAL_PROTOCOLPGM("SCARA Cal - Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS]+home_offset[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta (90):");
    SERIAL_PROTOCOL(delta[Y_AXIS]-delta[X_AXIS]-90+home_offset[Y_AXIS]);
    SERIAL_EOL;
    
    SERIAL_PROTOCOLPGM("SCARA step Cal - Theta:");
    SERIAL_PROTOCOL(delta[X_AXIS]/90*axis_steps_per_unit[X_AXIS]);
    SERIAL_PROTOCOLPGM("   Psi+Theta:");
    SERIAL_PROTOCOL((delta[Y_AXIS]-delta[X_AXIS])/90*axis_steps_per_unit[Y_AXIS]);
    SERIAL_EOL; SERIAL_EOL;
  #endif
#endif  
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  SERIAL_PROTOCOLPGM(MSG_M115_REPORT);
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() {
  char* codepos = strchr_pointer + 5;
  char* starpos = strchr(codepos, '*');
  if (starpos) *starpos = '\0';
  lcd_setstatus(codepos);
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() {
  SERIAL_PROTOCOLLN(MSG_M119_REPORT);
  #if HAS_X_MIN
    SERIAL_PROTOCOLPGM(MSG_X_MIN);
    SERIAL_PROTOCOLLN(((READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X_MAX
    SERIAL_PROTOCOLPGM(MSG_X_MAX);
    SERIAL_PROTOCOLLN(((READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MIN
    SERIAL_PROTOCOLPGM(MSG_Y_MIN);
    SERIAL_PROTOCOLLN(((READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MAX
    SERIAL_PROTOCOLPGM(MSG_Y_MAX);
    SERIAL_PROTOCOLLN(((READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MIN
    SERIAL_PROTOCOLPGM(MSG_Z_MIN);
    SERIAL_PROTOCOLLN(((READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MAX
    SERIAL_PROTOCOLPGM(MSG_Z_MAX);
    SERIAL_PROTOCOLLN(((READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MAX
    SERIAL_PROTOCOLPGM(MSG_Z2_MAX);
    SERIAL_PROTOCOLLN(((READ(Z2_MAX_PIN)^Z2_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_PROBE
    SERIAL_PROTOCOLPGM(MSG_Z_PROBE);
    SERIAL_PROTOCOLLN(((READ(Z_PROBE_PIN)^Z_PROBE_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_E_MIN
    SERIAL_PROTOCOLPGM(MSG_E_MIN);
    SERIAL_PROTOCOLLN(((READ(E_MIN_PIN)^E_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  //#if HAS_FILRUNOUT //aven_test0826
  //aven_0817 FILAMENT RUNOUT
    //SERIAL_PROTOCOLPGM(MSG_FILRUNOUT_PIN);
    SERIAL_PROTOCOLPGM(MSG_FILAMENT_RUNOUT_PIN);
    SERIAL_PROTOCOLLN(((READ(F0_STOP)^FIL_RUNOUT_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  	//SERIAL_PROTOCOLLN(((READ(FILRUNOUT_PIN)^FIL_RUNOUT_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
    //SERIAL_PROTOCOLLN(((READ(FILRUNOUT_PIN)^FILRUNOUT_PIN_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  //#endif
}

/**
 * M120: Enable endstops
 */
inline void gcode_M120() { enable_endstops(false); }

/**
 * M121: Disable endstops
 */
inline void gcode_M121() { enable_endstops(true); }

#ifdef BARICUDA
  #if HAS_HEATER_1
    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126() { ValvePressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { ValvePressure = 0; }
  #endif

  #if HAS_HEATER_2
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { EtoPPressure = code_seen('S') ? constrain(code_value(), 0, 255) : 255; }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { EtoPPressure = 0; }
  #endif

#endif //BARICUDA

/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
  if (debugDryrun()) return;
  if (code_seen('S')) setTargetBed(code_value());
}

#ifdef BLINKM
  /**
   * M150: Set Status LED Color - Use R-U-B for R-G-B
   */
  inline void gcode_M150() {
    SendColors(
      code_seen('R') ? (byte)code_value_short() : 0,
      code_seen('U') ? (byte)code_value_short() : 0,
      code_seen('B') ? (byte)code_value_short() : 0
    );
  }

#endif // BLINKM

#if HAS_TEMP_BED
  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (debugDryrun()) return;
    LCD_MESSAGEPGM(MSG_BED_HEATING);
    CooldownNoWait = code_seen('S');
    if (CooldownNoWait || code_seen('R'))
      setTargetBed(code_value());

    wait_bed();
  }
#endif // HAS_TEMP_BED

/**
 * M200: Set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
 *       T<extruder>
 *       D<millimeters>
 */
inline void gcode_M200() {
  int tmp_extruder = active_extruder;
  if (code_seen('T')) {
    tmp_extruder = code_value_short();
    if (tmp_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_ECHO(MSG_M200_INVALID_EXTRUDER);
      return;
    }
  }

  if (code_seen('D')) {
    float diameter = code_value();
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (diameter != 0.0);
    if (volumetric_enabled) {
      filament_size[tmp_extruder] = diameter;
      // make sure all extruders have some sane value for the filament size
      for (int i=0; i<EXTRUDERS; i++)
        if (! filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  else {
    //reserved for setting filament diameter via UFID or filament measuring device
    return;
  }
  calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 */
inline void gcode_M201() {
  for (int8_t i=0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      max_acceleration_units_per_sq_second[i] = code_value();
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    for(int8_t i=0; i < NUM_AXIS; i++) {
      if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
    }
  }
#endif


/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 */
inline void gcode_M203() {
  for (int8_t i=0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      max_feedrate[i] = code_value();
    }
  }
}

/**
 * M204: Set Accelerations in mm/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
  if (code_seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    acceleration = code_value();
    travel_acceleration = acceleration;
    SERIAL_ECHOPAIR("Setting Print and Travel Acceleration: ", acceleration );
    SERIAL_EOL;
  }
  if (code_seen('P')) {
    acceleration = code_value();
    SERIAL_ECHOPAIR("Setting Print Acceleration: ", acceleration );
    SERIAL_EOL;
  }
  if (code_seen('R')) {
    retract_acceleration = code_value();
    SERIAL_ECHOPAIR("Setting Retract Acceleration: ", retract_acceleration );
    SERIAL_EOL;
  }
  if (code_seen('T')) {
    travel_acceleration = code_value();
    SERIAL_ECHOPAIR("Setting Travel Acceleration: ", travel_acceleration );
    SERIAL_EOL;
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (mm/s)
 *    T = Min Travel Feed Rate (mm/s)
 *    B = Min Segment Time (µs)
 *    X = Max XY Jerk (mm/s/s)
 *    Z = Max Z Jerk (mm/s/s)
 *    E = Max E Jerk (mm/s/s)
 */
inline void gcode_M205() {
  if (code_seen('S')) minimumfeedrate = code_value();
  if (code_seen('T')) mintravelfeedrate = code_value();
  if (code_seen('B')) minsegmenttime = code_value();
  if (code_seen('X')) max_xy_jerk = code_value();
  if (code_seen('Z')) max_z_jerk = code_value();
  if (code_seen('E')) max_e_jerk = code_value();
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  for (int8_t i=X_AXIS; i <= Z_AXIS; i++) {
    if (code_seen(axis_codes[i])) {
      home_offset[i] = code_value();
    }
  }
  #ifdef SCARA
    if (code_seen('T')) home_offset[X_AXIS] = code_value(); // Theta
    if (code_seen('P')) home_offset[Y_AXIS] = code_value(); // Psi
  #endif
}

#ifdef FWRETRACT

  /**
   * M207: Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
   */
  inline void gcode_M207() {
    if (code_seen('S')) retract_length = code_value();
    if (code_seen('F')) retract_feedrate = code_value() / 60;
    if (code_seen('Z')) retract_zlift = code_value();
  }

  /**
   * M208: Set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
   */
  inline void gcode_M208() {
    if (code_seen('S')) retract_recover_length = code_value();
    if (code_seen('F')) retract_recover_feedrate = code_value() / 60;
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *       detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
   */
  inline void gcode_M209() {
    if (code_seen('S')) {
      int t = code_value_short();
      switch(t) {
        case 0:
          autoretract_enabled = false;
          break;
        case 1:
          autoretract_enabled = true;
          break;
        default:
          SERIAL_ECHO_START;
          SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
          SERIAL_ECHO(cmdbuffer[bufindr]);
          SERIAL_ECHOLNPGM("\"");
          return;
      }
      for (int i=0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }
#endif // FWRETRACT

#if HOTENDS > 1

  /**
   * M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
   */
  inline void gcode_M218() {
    if (setTargetedHotend(218)) return;

    if (code_seen('X')) hotend_offset[X_AXIS][target_extruder] = code_value();
    if (code_seen('Y')) hotend_offset[Y_AXIS][target_extruder] = code_value();

    #ifdef DUAL_X_CARRIAGE
      if (code_seen('Z')) hotend_offset[Z_AXIS][target_extruder] = code_value();
    #endif

    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
    for (int e = 0; e < EXTRUDERS; e++) {
      SERIAL_CHAR(' ');
      SERIAL_ECHO(hotend_offset[X_AXIS][e]);
      SERIAL_CHAR(',');
      SERIAL_ECHO(hotend_offset[Y_AXIS][e]);
      #ifdef DUAL_X_CARRIAGE
        SERIAL_CHAR(',');
        SERIAL_ECHO(hotend_offset[Z_AXIS][e]);
      #endif
    }
    SERIAL_EOL;
  }
#endif //HOTENDS > 1

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (code_seen('S')) feedmultiply = code_value();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
  if (code_seen('S')) {
    int sval = code_value();
    if (code_seen('T')) {
      if (setTargetedHotend(221)) return;
      extruder_multiply[target_extruder] = sval;
    }
    else {
      extruder_multiply[active_extruder] = sval;
    }
  }
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (code_seen('P')) {
    int pin_number = code_value();

    int pin_state = code_seen('S') ? code_value() : -1; // required pin state - default is inverted

    if (pin_state >= -1 && pin_state <= 1) {

      for (int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(*sensitive_pins)); i++) {
        if (sensitive_pins[i] == pin_number) {
          pin_number = -1;
          break;
        }
      }

      if (pin_number > -1) {
        int target = LOW;

        st_synchronize();

        pinMode(pin_number, INPUT);

        switch(pin_state){
          case 1:
            target = HIGH;
            break;

          case 0:
            target = LOW;
            break;

          case -1:
            target = !digitalRead(pin_number);
            break;
        }

        while(digitalRead(pin_number) != target) {
          manage_heater();
          manage_inactivity();
          lcd_update();
        }

      } // pin_number > -1
    } // pin_state -1 0 1
  } // code_seen('P')
}

#if defined(CHDK) || HAS_PHOTOGRAPH
  /**
   * M240: Trigger a camera by emulating a Canon RC-1
   *       See http://www.doc-diy.net/photo/rc-1_hacked/
   */
  inline void gcode_M240() {
    #ifdef CHDK
       OUT_WRITE(CHDK, HIGH);
       chdkHigh = millis();
       chdkActive = true;
    #elif HAS_PHOTOGRAPH
      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
      delay(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
      }
    #endif // !CHDK && HAS_PHOTOGRAPH
  }
#endif // CHDK || PHOTOGRAPH_PIN

#if defined(DOGLCD) && LCD_CONTRAST >= 0
  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (code_seen('C')) lcd_setcontrast(code_value_short() & 0x3F);
    SERIAL_PROTOCOLPGM("lcd contrast value: ");
    SERIAL_PROTOCOL(lcd_contrast);
    SERIAL_EOL;
  }

#endif // DOGLCD

#if NUM_SERVOS > 0
  /**
   * M280: Set servo position absolute. P: servo index, S: angle or microseconds
   */
  inline void gcode_M280() {
    int servo_index = code_seen('P') ? code_value() : -1;
    int servo_position = 0;
    if (code_seen('S')) {
      servo_position = code_value();
      if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
        #if SERVO_LEVELING
          servos[servo_index].attach(0);
        #endif
        servos[servo_index].write(servo_position);
        #if SERVO_LEVELING
          delay(PROBE_SERVO_DEACTIVATION_DELAY);
          servos[servo_index].detach();
        #endif
      }
      else {
        SERIAL_ECHO_START;
        SERIAL_ECHO("Servo ");
        SERIAL_ECHO(servo_index);
        SERIAL_ECHOLN(" out of range");
      }
    }
    else if (servo_index >= 0) {
      SERIAL_PROTOCOL(MSG_OK);
      SERIAL_PROTOCOL(" Servo ");
      SERIAL_PROTOCOL(servo_index);
      SERIAL_PROTOCOL(": ");
      SERIAL_PROTOCOL(servos[servo_index].read());
      SERIAL_EOL;
    }
  }
#endif // NUM_SERVOS > 0

#if defined(LARGE_FLASH) && (BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER))
  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    int beepS = code_seen('S') ? code_value() : 110;
    int beepP = code_seen('P') ? code_value() : 1000;
    if (beepS > 0) {
      #if BEEPER > 0
        tone(BEEPER, beepS);
        delay(beepP);
        noTone(BEEPER);
      #elif defined(ULTRALCD)
        lcd_buzz(beepS, beepP);
      #elif defined(LCD_USE_I2C_BUZZER)
        lcd_buzz(beepP, beepS);
      #endif
    }
    else {
      delay(beepP);
    }
  }
#endif // LARGE_FLASH && (BEEPER>0 || ULTRALCD || LCD_USE_I2C_BUZZER)

#ifdef PIDTEMP
  /**
   * M301: Set PID parameters P I D
   */
  inline void gcode_M301() {

    // multi-hotend PID patch: M301 updates or prints a single hotend's PID values
    // default behaviour (omitting E parameter) is to update for hotend 0 only
    int e = code_seen('E') ? code_value() : 0; // hotend being updated

    if (e < HOTENDS) { // catch bad input value
      if (code_seen('P')) PID_PARAM(Kp, e) = code_value();
      if (code_seen('I')) PID_PARAM(Ki, e) = scalePID_i(code_value());
      if (code_seen('D')) PID_PARAM(Kd, e) = scalePID_d(code_value());

      updatePID();
      SERIAL_PROTOCOL(MSG_OK);
      SERIAL_PROTOCOL(" e:"); // specify hotend in serial output
      SERIAL_PROTOCOL(e);
      SERIAL_PROTOCOL(" p:");
      SERIAL_PROTOCOL(PID_PARAM(Kp, e));
      SERIAL_PROTOCOL(" i:");
      SERIAL_PROTOCOL(unscalePID_i(PID_PARAM(Ki, e)));
      SERIAL_PROTOCOL(" d:");
      SERIAL_PROTOCOL(unscalePID_d(PID_PARAM(Kd, e)));
      SERIAL_EOL;    
    }
    else {
      SERIAL_ECHO_START;
      SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
    }
  }
#endif // PIDTEMP

#ifdef PREVENT_DANGEROUS_EXTRUDE
  /**
   * M302: Allow cold extrudes, or set the minimum extrude S<temperature>.
   */
  inline void gcode_M302() {
    set_extrude_min_temp(code_seen('S') ? code_value() : 0);
  }

#endif // PREVENT_DANGEROUS_EXTRUDE

/**
 * M303: PID relay autotune
 *       S<temperature> sets the target temperature. (default target temperature = 150C)
 *       E<extruder> (-1 for the bed)
 *       C<cycles>
 */
inline void gcode_M303() {
  int e = code_seen('E') ? code_value_short() : 0;
  int c = code_seen('C') ? code_value_short() : 5;
  float temp = code_seen('S') ? code_value() : (e < 0 ? 70.0 : 150.0);
  PID_autotune(temp, e, c);
}

#ifdef PIDTEMPBED
  // M304: Set bed PID parameters P I and D
  inline void gcode_M304() {
    if (code_seen('P')) bedKp = code_value();
    if (code_seen('I')) bedKi = scalePID_i(code_value());
    if (code_seen('D')) bedKd = scalePID_d(code_value());

    updatePID();
    SERIAL_PROTOCOL(MSG_OK);
    SERIAL_PROTOCOL(" p:");
    SERIAL_PROTOCOL(bedKp);
    SERIAL_PROTOCOL(" i:");
    SERIAL_PROTOCOL(unscalePID_i(bedKi));
    SERIAL_PROTOCOL(" d:");
    SERIAL_PROTOCOL(unscalePID_d(bedKd));
    SERIAL_EOL;
  }
#endif // PIDTEMPBED

#if HAS_MICROSTEPS
  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
    for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
    if(code_seen('B')) microstep_mode(4,code_value());
    microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (code_seen('S')) switch(code_value_short()) {
      case 1:
        for(int i=0;i<NUM_AXIS;i++) if (code_seen(axis_codes[i])) microstep_ms(i, code_value(), -1);
        if (code_seen('B')) microstep_ms(4, code_value(), -1);
        break;
      case 2:
        for(int i=0;i<NUM_AXIS;i++) if (code_seen(axis_codes[i])) microstep_ms(i, -1, code_value());
        if (code_seen('B')) microstep_ms(4, -1, code_value());
        break;
    }
    microstep_readings();
  }
#endif // HAS_MICROSTEPS

#ifdef SCARA
  bool SCARA_move_to_cal(uint8_t delta_x, uint8_t delta_y) {
    //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
    //SERIAL_ECHOLN(" Soft endstops disabled ");
    if (! Stopped) {
      //get_coordinates(); // For X Y Z E F
      delta[X_AXIS] = delta_x;
      delta[Y_AXIS] = delta_y;
      calculate_SCARA_forward_Transform(delta);
      destination[X_AXIS] = delta[X_AXIS]/axis_scaling[X_AXIS];
      destination[Y_AXIS] = delta[Y_AXIS]/axis_scaling[Y_AXIS];
      prepare_move();
      //ClearToSend();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    SERIAL_ECHOLN(" Cal: Theta 0 ");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    SERIAL_ECHOLN(" Cal: Theta 90 ");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    SERIAL_ECHOLN(" Cal: Psi 0 ");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    SERIAL_ECHOLN(" Cal: Psi 90 ");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    SERIAL_ECHOLN(" Cal: Theta-Psi 90 ");
    return SCARA_move_to_cal(45, 135);
  }

  /**
   * M365: SCARA calibration: Scaling factor, X, Y, Z axis
   */
  inline void gcode_M365() {
    for (int8_t i = X_AXIS; i <= Z_AXIS; i++) {
      if (code_seen(axis_codes[i])) {
        axis_scaling[i] = code_value();
      }
    }
  }
#endif // SCARA

#ifdef EXT_SOLENOID
  void enable_solenoid(uint8_t num) {
    switch(num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS_SOLENOID_1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS_SOLENOID_3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_INVALID_SOLENOID);
        break;
    }
  }

  void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }

  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    OUT_WRITE(SOL1_PIN, LOW);
    OUT_WRITE(SOL2_PIN, LOW);
    OUT_WRITE(SOL3_PIN, LOW);
  }

  /**
   * M380: Enable solenoid on the active extruder
   */
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

  /**
   * M381: Disable all solenoids
   */
  inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { st_synchronize(); }

#if defined(ENABLE_AUTO_BED_LEVELING) && defined(SERVO_ENDSTOPS) && (NUM_SERVOS > 0) && not defined(Z_PROBE_SLED)

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() { deploy_z_probe(); }
  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() { stow_z_probe(); }

#endif

#ifdef FILAMENT_SENSOR

  /**
   * M404: Display or set the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    #if HAS_FILWIDTH
      if (code_seen('D')) {
        filament_width_nominal = code_value();
      }
      else {
        SERIAL_PROTOCOLPGM("Filament dia (nominal mm):");
        SERIAL_PROTOCOLLN(filament_width_nominal);
      }
    #endif
  }
    
  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405() {
    if (code_seen('D')) meas_delay_cm = code_value();
    if (meas_delay_cm > MAX_MEASUREMENT_DELAY) meas_delay_cm = MAX_MEASUREMENT_DELAY;

    if (delay_index2 == -1) { //initialize the ring buffer if it has not been done since startup
      int temp_ratio = widthFil_to_size_ratio();

      for (delay_index1 = 0; delay_index1 < MAX_MEASUREMENT_DELAY + 1; ++delay_index1)
        measurement_delay[delay_index1] = temp_ratio - 100;  //subtract 100 to scale within a signed byte

      delay_index1 = delay_index2 = 0;
    }

    filament_sensor = true;

    //SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
    //SERIAL_PROTOCOL(filament_width_meas);
    //SERIAL_PROTOCOLPGM("Extrusion ratio(%):");
    //SERIAL_PROTOCOL(extruder_multiply[active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406() { filament_sensor = false; }
  
  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407() {
    SERIAL_PROTOCOLPGM("Filament dia (measured mm):"); 
    SERIAL_PROTOCOLLN(filament_width_meas);   
  }

#endif // FILAMENT_SENSOR

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  Config_StoreSettings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  Config_RetrieveSettings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  Config_ResetDefault();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  Config_PrintSettings(code_seen('S') && code_value() == 0);
}

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (code_seen('S')) abort_on_endstop_hit = (code_value() > 0);
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

//#ifdef FILAMENTCHANGEENABLE //aven_test0826
  /**
   * M600: Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
   */
  inline void gcode_M600() {
    float target[NUM_AXIS], lastpos[NUM_AXIS], fr60 = feedrate / 60;
    filament_changing = true;
    for (int i=0; i < NUM_AXIS; i++) target[i] = lastpos[i] = current_position[i];

    #define BASICPLAN plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], fr60, active_extruder, active_driver);
    #ifdef DELTA
      #define RUNPLAN calculate_delta(target); BASICPLAN
    #else
      #define RUNPLAN BASICPLAN
    #endif

    //retract by E
    if (code_seen('E')) target[E_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_FIRSTRETRACT
      else target[E_AXIS] += FILAMENTCHANGE_FIRSTRETRACT;
    #endif

    RUNPLAN;

    //lift Z
    if (code_seen('Z')) target[Z_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_ZADD
      else target[Z_AXIS] += FILAMENTCHANGE_ZADD;
    #endif

    RUNPLAN;

    //move xy
    if (code_seen('X')) target[X_AXIS] = code_value();
    #ifdef FILAMENTCHANGE_XPOS
      else target[X_AXIS] = FILAMENTCHANGE_XPOS;
    #endif

    if (code_seen('Y')) target[Y_AXIS] = code_value();
    #ifdef FILAMENTCHANGE_YPOS
      else target[Y_AXIS] = FILAMENTCHANGE_YPOS;
    #endif

    RUNPLAN;

    if (code_seen('L')) target[E_AXIS] += code_value();
    #ifdef FILAMENTCHANGE_FINALRETRACT
      else target[E_AXIS] += FILAMENTCHANGE_FINALRETRACT;
    #endif

    RUNPLAN;

    //finish moves
    st_synchronize();
    //disable extruder steppers so filament can be removed
    disable_e();
    delay(100);
    boolean beep = true;
    boolean sleep = false;
    int cnt = 0;
    
    int old_target_temperature[HOTENDS] = { 0 };
    for(int8_t e = 0; e < HOTENDS; e++)
    {
      old_target_temperature[e] = target_temperature[e];
    }
    int old_target_temperature_bed = target_temperature_bed;
    timer.set_max_delay(60000); // 1 minute

#if 0	//aven_test0826
    PRESSBUTTON:	
    LCD_ALERTMESSAGEPGM(MSG_FILAMENTCHANGE);
    while (!lcd_clicked()) {
      manage_heater();
      manage_inactivity(true);
      lcd_update();
      if (timer.check() && cnt <= 5) beep = true;
      if (cnt >= 5 && !sleep) {
        disable_heater();
        disable_x();
        disable_y();
        disable_z();
        disable_e();    
        sleep = true;
        lcd_reset_alert_level();
        LCD_ALERTMESSAGEPGM("Zzzz Zzzz Zzzz");
      }
      if (beep) {
        timer.set();        
        lcd_beep(3);
        beep = false;
        cnt += 1;
      }
    }

    //reset LCD alert message
    lcd_reset_alert_level();
#endif	

    if (sleep) {
      for(int8_t e = 0; e < HOTENDS; e++)
      {
        setTargetHotend(old_target_temperature[e], e);
        CooldownNoWait = true;
        wait_heater();
      }
      setTargetBed(old_target_temperature_bed);
      CooldownNoWait = true;
      wait_bed();
      sleep = false;
      beep = true;
      cnt = 0;
      //goto PRESSBUTTON;//aven_test0826
    }

    //return to normal
    if (code_seen('L')) target[E_AXIS] -= code_value();
    #ifdef FILAMENTCHANGE_FINALRETRACT
      else target[E_AXIS] -= FILAMENTCHANGE_FINALRETRACT;
    #endif

    #ifdef FILAMENT_END_SWITCH
      paused = false;
    #endif

    for(int8_t i=0; i < NUM_AXIS; i++) current_position[i]=target[i];
    sync_plan_position();

    // HOME X & Y & Z(only Delta)
    //gcode_G28(true,true); //aven_tets0826
    gcode_G28();
    #ifdef DELTA
      calculate_delta(lastpos);
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder, active_driver); //move xyz back
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder, active_driver); //final unretract
      for(int8_t i=0; i < NUM_AXIS; i++) current_position[i]=lastpos[i];
      calculate_delta(current_position);
      plan_set_position(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS]);
    #else
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder, active_driver); //move xy back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], feedrate/60, active_extruder, active_driver); //move z back
      plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], lastpos[E_AXIS], feedrate/60, active_extruder, active_driver); //final unretract
      for(int8_t i=0; i < NUM_AXIS; i++) current_position[i]=lastpos[i];
      sync_plan_position();
    #endif
    filament_changing = false;
  }
//#endif //FILAMENTCHANGEENABLE

#ifdef DUAL_X_CARRIAGE
  /**
   * M605: Set dual x-carriage movement mode
   *
   *    M605 S0: Full control mode. The slicer has full control over x-carriage movement
   *    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
   *    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
   *                         millimeters x-offset and an optional differential hotend temperature of
   *                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
   *                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
   *
   *    Note: the X axis should be homed after changing dual x-carriage mode.
   */
  inline void gcode_M605() {
    st_synchronize();
    if (code_seen('S')) dual_x_carriage_mode = code_value();
    switch(dual_x_carriage_mode) {
      case DXC_DUPLICATION_MODE:
        if (code_seen('X')) duplicate_extruder_x_offset = max(code_value(), X2_MIN_POS - x_home_pos(0));
        if (code_seen('R')) duplicate_extruder_temp_offset = code_value();
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM(MSG_HOTEND_OFFSET);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(extruder_offset[X_AXIS][0]);
        SERIAL_CHAR(',');
        SERIAL_ECHO(extruder_offset[Y_AXIS][0]);
        SERIAL_CHAR(' ');
        SERIAL_ECHO(duplicate_extruder_x_offset);
        SERIAL_CHAR(',');
        SERIAL_ECHOLN(extruder_offset[Y_AXIS][1]);
        break;
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      default:
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    active_extruder_parked = false;
    extruder_duplication_enabled = false;
    delayed_move_time = 0;
  }
#endif // DUAL_X_CARRIAGE

#ifdef ENABLE_AUTO_BED_LEVELING
  //M666: Set Z probe offset
  inline void gcode_M666() {
    if (code_seen('P')) {
      zprobe_zoffset = code_value();
    }
    if (code_seen('L')) {
      SERIAL_ECHOPAIR("P (Z-Probe Offset):", zprobe_zoffset);
      SERIAL_EOL;
    }
  }
#endif

#ifdef DELTA
  //M666: Set delta endstop and geometry adjustment
  inline void gcode_M666() {
    if ( !(code_seen('P'))) {
      for(int8_t i=0; i < 3; i++) {
        if (code_seen(axis_codes[i])) endstop_adj[i] = code_value();
      }
    }
    if (code_seen('A')) {
      tower_adj[0] = code_value();
      set_delta_constants();
    }
    if (code_seen('B')) {
      tower_adj[1] = code_value();
      set_delta_constants();
    }
    if (code_seen('C')) {
      tower_adj[2] = code_value();
      set_delta_constants();
    }
    if (code_seen('I')) {
      tower_adj[3] = code_value();
      set_delta_constants();
    }
    if (code_seen('J')) {
      tower_adj[4] = code_value();
      set_delta_constants();
    }
    if (code_seen('K')) {
      tower_adj[5] = code_value();
      set_delta_constants();
    }
    if (code_seen('R')) {
      delta_radius = code_value();
      set_delta_constants();
    }
    if (code_seen('D')) {
      delta_diagonal_rod = code_value();
      set_delta_constants();
    }
    if (code_seen('H')) {
      max_pos[Z_AXIS]= code_value();
      set_delta_constants();
    }
    if (code_seen('P')) {
      float pz = code_value();
      if (!(code_seen(axis_codes[0]) || code_seen(axis_codes[1]) || code_seen(axis_codes[2]))) {  // Allow direct set of Z offset without an axis code
        z_probe_offset[Z_AXIS]= pz;
      }
      else {
        for(int8_t i=0; i < 3; i++) {
          if (code_seen(axis_codes[i])) z_probe_offset[i] = code_value();
        }
      }
    }
    if (code_seen('L')) {
      SERIAL_ECHOLN("Current Delta geometry values:");
      SERIAL_ECHOPAIR("X (Endstop Adj): ",endstop_adj[0]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("Y (Endstop Adj): ",endstop_adj[1]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("Z (Endstop Adj): ",endstop_adj[2]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("P (Z-Probe Offset): X", z_probe_offset[0]);
      SERIAL_ECHOPAIR(" Y", z_probe_offset[1]);
      SERIAL_ECHOPAIR(" Z", z_probe_offset[2]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("A (Tower A Position Correction): ",tower_adj[0]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("B (Tower B Position Correction): ",tower_adj[1]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("C (Tower C Position Correction): ",tower_adj[2]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("I (Tower A Radius Correction): ",tower_adj[3]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("J (Tower B Radius Correction): ",tower_adj[4]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("K (Tower C Radius Correction): ",tower_adj[5]);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("R (Delta Radius): ",delta_radius);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("D (Diagonal Rod Length): ",delta_diagonal_rod);
      SERIAL_EOL;
      SERIAL_ECHOPAIR("H (Z-Height): ",max_pos[Z_AXIS]);
      SERIAL_EOL;
    }
  }
#endif

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    for (int i=0;i<NUM_AXIS;i++)
      if (code_seen(axis_codes[i])) digipot_current(i, code_value());
    if (code_seen('B')) digipot_current(4, code_value());
    if (code_seen('S')) for (int i=0; i<=4; i++) digipot_current(i, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_XY_PIN
    if (code_seen('X')) digipot_current(0, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_Z_PIN
    if (code_seen('Z')) digipot_current(1, code_value());
  #endif
  #ifdef MOTOR_CURRENT_PWM_E_PIN
    if (code_seen('E')) digipot_current(2, code_value());
  #endif
  #ifdef DIGIPOT_I2C
    // this one uses actual amps in floating point
    for (int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (int i=NUM_AXIS;i<DIGIPOT_I2C_NUM_CHANNELS;i++) if(code_seen('B'+i-NUM_AXIS)) digipot_i2c_set_current(i, code_value());
  #endif
}

#if HAS_DIGIPOTSS
  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    digitalPotWrite(
      code_seen('P') ? code_value() : 0,
      code_seen('S') ? code_value() : 0
    );
  }
#endif // HAS_DIGIPOTSS

#ifdef NPR2
  /**
   * M997: Cxx Move Carter xx gradi
   */
  inline void gcode_M997() {
    long csteps;
    if (code_seen('C')) {
      csteps = code_value() * color_step_moltiplicator;
      SERIAL_ECHO_START;
      SERIAL_ECHO("csteps: ");
      SERIAL_PROTOCOLLN(csteps);
      if (csteps < 0) colorstep(-csteps,false);
      if (csteps > 0) colorstep(csteps,true);
    }
  }
#endif

/**
 * M999: Restart after being stopped
 */
inline void gcode_M999() {
  Stopped = false;
  lcd_reset_alert_level();
  gcode_LastN = Stopped_gcode_LastN;
  FlushSerialRequestResend();
}

#ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
  inline void gcode_SET_Z_PROBE_OFFSET() {
    float value;
    if (code_seen('Z')) {
      value = code_value();
      if (Z_PROBE_OFFSET_RANGE_MIN <= value && value <= Z_PROBE_OFFSET_RANGE_MAX) {
        zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
        SERIAL_ECHO_START;
        SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " " MSG_OK);
        SERIAL_EOL;
      }
      else {
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM(MSG_ZPROBE_ZOFFSET);
        SERIAL_ECHOPGM(MSG_Z_MIN);
        SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
        SERIAL_ECHOPGM(MSG_Z_MAX);
        SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
        SERIAL_EOL;
      }
    }
    else {
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_ZPROBE_ZOFFSET " : ");
      SERIAL_ECHO(-zprobe_zoffset);
      SERIAL_EOL;
    }
  }

#endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

inline void gcode_T() {
  int tmp_extruder = code_value();
  long csteps;
  if (tmp_extruder >= EXTRUDERS) {
    SERIAL_ECHO_START;
    SERIAL_CHAR('T');
    SERIAL_ECHO(tmp_extruder);
    SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
  }
  else {
    target_extruder = tmp_extruder;

    #if EXTRUDERS > 1
      bool make_move = false;
    #endif

    if (code_seen('F')) {

      #if EXTRUDERS > 1
        make_move = true;
      #endif

      next_feedrate = code_value();
      if (next_feedrate > 0.0) feedrate = next_feedrate;
    }
    #if EXTRUDERS > 1
      #ifdef NPR2
        if(target_extruder != old_color)
      #else
        if(target_extruder != active_extruder)
      #endif // NPR2
      {
        // Save current position to return to after applying extruder offset
        set_destination_to_current();
        #ifdef DUAL_X_CARRIAGE
          if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && Stopped == false &&
            (delayed_move_time != 0 || current_position[X_AXIS] != x_home_pos(active_extruder)))
          {
            // Park old head: 1) raise 2) move to park position 3) lower
            plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT, current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
            plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT, current_position[E_AXIS], max_feedrate[X_AXIS], active_extruder, active_driver);
            plan_buffer_line(x_home_pos(active_extruder), current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
            st_synchronize();
          }

          // apply Y & Z extruder offset (x offset is already used in determining home pos)
          current_position[Y_AXIS] = current_position[Y_AXIS] -
                        hotend_offset[Y_AXIS][active_extruder] +
                        hotend_offset[Y_AXIS][target_extruder];
          current_position[Z_AXIS] = current_position[Z_AXIS] -
                        hotend_offset[Z_AXIS][active_extruder] +
                        hotend_offset[Z_AXIS][target_extruder];

          active_extruder = target_extruder;

          // This function resets the max/min values - the current position may be overwritten below.
          axis_is_at_home(X_AXIS);

          if (dual_x_carriage_mode == DXC_FULL_CONTROL_MODE) {
            current_position[X_AXIS] = inactive_extruder_x_pos;
            inactive_extruder_x_pos = destination[X_AXIS];
          }
          else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
            active_extruder_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
            if (active_extruder == 0 || active_extruder_parked)
              current_position[X_AXIS] = inactive_extruder_x_pos;
            else
              current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
            inactive_extruder_x_pos = destination[X_AXIS];
            extruder_duplication_enabled = false;
          }
          else {
            // record raised toolhead position for use by unpark
            memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
            raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
            active_extruder_parked = true;
            delayed_move_time = 0;
          }
        #else // !DUAL_X_CARRIAGE
          // Offset hotend (only by XY)
          #if HOTENDS > 1
            for (int i=X_AXIS; i<=Y_AXIS; i++)
              current_position[i] += hotend_offset[i][target_extruder] - hotend_offset[i][active_extruder];
          #endif // HOTENDS > 1

          #if defined(MKR4) && (EXTRUDERS > 1)
            #if (EXTRUDERS == 4) && (E0E2_CHOICE_PIN >1) && (E1E3_CHOICE_PIN > 1) && (DRIVER_EXTRUDERS == 2)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E2_CHOICE_PIN,LOW);
                WRITE(E1E3_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E2_CHOICE_PIN,LOW);
                WRITE(E1E3_CHOICE_PIN,LOW);
                active_driver = 1;
                delay(500); // 500 microseconds delay for relay
                enable_e1();             
                break;
              case 2:
                WRITE(E0E2_CHOICE_PIN,HIGH);
                WRITE(E1E3_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e2();
                break;
              case 3:
                WRITE(E0E2_CHOICE_PIN,LOW);
                WRITE(E1E3_CHOICE_PIN,HIGH);
                active_driver = 1;
                delay(500); // 500 microseconds delay for relay
                enable_e3();
                break;
              }            
            #elif (EXTRUDERS == 3) && (E0E2_CHOICE_PIN >1) && (DRIVER_EXTRUDERS == 2)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 1;
                delay(500); // 500 microseconds delay for relay
                enable_e1();
                break;
              case 2:
                WRITE(E0E2_CHOICE_PIN,HIGH);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e2();
                break;
              }
            #elif (EXTRUDERS == 3) && (E0E1_CHOICE_PIN >1) && (E0E2_CHOICE_PIN >1) && (DRIVER_EXTRUDERS == 1)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E1_CHOICE_PIN,LOW);
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E1_CHOICE_PIN,HIGH);
                WRITE(E0E2_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 2:
                WRITE(E0E1_CHOICE_PIN,HIGH);
                WRITE(E0E2_CHOICE_PIN,HIGH);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              }
            #elif (EXTRUDERS == 2) && (E0E1_CHOICE_PIN >1) && (DRIVER_EXTRUDERS == 1)
              st_synchronize(); // Finish all movement
              disable_e();
              switch(target_extruder)
              {
              case 0:
                WRITE(E0E1_CHOICE_PIN,LOW);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              case 1:
                WRITE(E0E1_CHOICE_PIN,HIGH);
                active_driver = 0;
                delay(500); // 500 microseconds delay for relay
                enable_e0();
                break;
              }
            #endif // E0E1_CHOICE_PIN E0E2_CHOICE_PIN E1E3_CHOICE_PIN
            active_extruder = target_extruder;
            SERIAL_ECHO_START;
            SERIAL_ECHO("Active Driver: ");
            SERIAL_PROTOCOLLN((int)active_driver);
            SERIAL_ECHO_START;
            SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
            SERIAL_PROTOCOLLN((int)active_extruder);
          #elif defined(NPR2)
            st_synchronize(); // Finish all movement
            if (old_color == 99)
            {
              csteps = (color_position[target_extruder]) * color_step_moltiplicator;
            }
            else
            {
              csteps = (color_position[target_extruder] - color_position[old_color]) * color_step_moltiplicator;
            }
            if (csteps < 0) colorstep(-csteps,false);
            if (csteps > 0) colorstep(csteps,true);
            old_color = active_extruder = target_extruder;
            active_driver = 0;
            SERIAL_ECHO_START;
            SERIAL_ECHO("Active Color: ");
            SERIAL_PROTOCOLLN((int)active_extruder);
          #else 
            active_driver = active_extruder = target_extruder;
            SERIAL_ECHO_START;
            SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
            SERIAL_PROTOCOLLN((int)active_extruder);

          #endif // end MKR4 || NPR2
        #endif // end no DUAL_X_CARRIAGE

        #ifdef DELTA 
          sync_plan_position_delta();
        #else // NO DELTA
          sync_plan_position();
        #endif // DELTA
        // Move to the old position if 'F' was in the parameters
        if (make_move && !Stopped) prepare_move();
      }

      #ifdef EXT_SOLENOID
        st_synchronize();
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

    #endif // EXTRUDERS > 1
  }
}


#if 0
/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 * If numbers are included with XYZ set the position as with G92
 * Currently adds the home_offset, which may be wrong and removed soon.
 *
 *  Xn  Home X, setting X to n + home_offset[X_AXIS]
 *  Yn  Home Y, setting Y to n + home_offset[Y_AXIS]
 *  Zn  Home Z, setting Z to n + home_offset[Z_AXIS]
 */
inline void gcode_G28(boolean home_x = false, boolean home_y = false) 
{

  G28_f = 1;//aven_0807
  
  #ifdef ENABLE_AUTO_BED_LEVELING
    plan_bed_level_matrix.set_to_identity();
  #endif //ENABLE_AUTO_BED_LEVELING

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  refresh_cmd_timeout();

  enable_endstops(true);

  set_destination_to_current();

  feedrate = 0.0;

  bool  homeX = code_seen(axis_codes[X_AXIS]),
        homeY = code_seen(axis_codes[Y_AXIS]),
        homeZ = code_seen(axis_codes[Z_AXIS]),
        homeE = code_seen(axis_codes[E_AXIS]);
        
  home_all_axis = !(homeX || homeY || homeZ || homeE || home_x || home_y) || (homeX && homeY && homeZ);

#if 0
  #ifdef NPR2
    if((home_all_axis) || (code_seen(axis_codes[E_AXIS]))) {
      active_driver = active_extruder = 1;
      plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], -200, COLOR_HOMERATE, active_extruder, active_driver);
      st_synchronize();
      old_color = 99;
      active_driver = active_extruder = 0;
    }
  #endif
#endif

  #ifdef DELTA
    // A delta can only safely home all axis at the same time
    // all axis have to home at the same time

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
    st_synchronize();
    endstops_hit_on_purpose(); // clear endstop hit flags

    // Destination reached
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = destination[i];
    // take care of back off and rehome now we are all at the top
    
    HOMEAXIS(X);
	
    HOMEAXIS(Y);
	
    HOMEAXIS(Z);

    sync_plan_position_delta();

  #endif //DELTA

#if 0 //aven_0807
  #if defined(CARTESIAN) || defined(COREXY) || defined(SCARA)

    #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if (home_all_axis || homeZ) HOMEAXIS(Z);
    #elif !defined(Z_SAFE_HOMING) && defined(Z_RAISE_BEFORE_HOMING) && Z_RAISE_BEFORE_HOMING > 0
      // Raise Z before homing any other axes
      if (home_all_axis || homeZ) {
        destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
        feedrate = max_feedrate[Z_AXIS] * 60;
        line_to_destination();
        st_synchronize();
      }
    #endif

    #ifdef QUICK_HOME
      if (home_all_axis || (homeX && homeY)) {  // First diagonal move

        current_position[X_AXIS] = current_position[Y_AXIS] = 0;

        #ifdef DUAL_X_CARRIAGE
          int x_axis_home_dir = x_home_dir(active_extruder);
          extruder_duplication_enabled = false;
        #else
          int x_axis_home_dir = home_dir(X_AXIS);
        #endif

        sync_plan_position();

        float mlx = max_length(X_AXIS), mly = max_length(Y_AXIS),
              mlratio = mlx>mly ? mly/mlx : mlx/mly;

        destination[X_AXIS] = 1.5 * mlx * x_axis_home_dir;
        destination[Y_AXIS] = 1.5 * mly * home_dir(Y_AXIS);
        feedrate = min(homing_feedrate[X_AXIS], homing_feedrate[Y_AXIS]) * sqrt(mlratio * mlratio + 1);
        line_to_destination();
        st_synchronize();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        sync_plan_position();

        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        line_to_destination();
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose(); // clear endstop hit flags

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        #ifndef SCARA
          current_position[Z_AXIS] = destination[Z_AXIS];
        #endif
      }
    #endif // QUICK_HOME

    if (home_all_axis || homeX) {
      #ifdef DUAL_X_CARRIAGE
        int tmp_extruder = active_extruder;
        extruder_duplication_enabled = false;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_extruder_x_pos = current_position[X_AXIS];
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif //DUAL_X_CARRIAGE
    }

    // Home Y
    if (home_all_axis || homeY) HOMEAXIS(Y);

    // Set the X position, if included
    // Adds the home_offset as well, which may be wrong
    if (homeX) {
      float v = code_value();
      if (v) current_position[X_AXIS] = v
        #ifndef SCARA
          + home_offset[X_AXIS]
        #endif
      ;
    }

    // Set the Y position, if included
    // Adds the home_offset as well, which may be wrong
    if (homeY) {
      float v = code_value();
      if (v) current_position[Y_AXIS] = v
        #ifndef SCARA
          + home_offset[Y_AXIS]
        #endif
      ;
    }

    #if Z_HOME_DIR < 0  // If homing towards BED do Z last
      #ifndef Z_SAFE_HOMING
        if (code_seen('M') && !(homeX || homeY)) {
          // Manual G28 bed level
          #ifdef ULTIPANEL
            SERIAL_ECHOLN(" --LEVEL PLATE SCRIPT--");
            set_ChangeScreen(true);
            while(!lcd_clicked()) {
              set_pageShowInfo(0);
              lcd_update();
            }
            saved_feedrate = feedrate;
            saved_feedmultiply = feedmultiply;
            feedmultiply = 100;
            previous_millis_cmd = millis();

            enable_endstops(true);
            for(int8_t i=0; i < NUM_AXIS; i++) {
              destination[i] = current_position[i];
            }
            feedrate = 0.0;
            #if Z_HOME_DIR > 0  // If homing away from BED do Z first
              HOMEAXIS(Z);
            #endif
            HOMEAXIS(X);
            HOMEAXIS(Y);
            #if Z_HOME_DIR < 0
              HOMEAXIS(Z);
            #endif
            sync_plan_position();

            #ifdef ENDSTOPS_ONLY_FOR_HOMING
              enable_endstops(false);
            #endif

            feedrate = saved_feedrate;
            feedmultiply = saved_feedmultiply;
            previous_millis_cmd = millis();
            endstops_hit_on_purpose(); // clear endstop hit flags

            sync_plan_position();

            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS + 5);

            // PROBE FIRST POINT
            set_pageShowInfo(1);
            set_ChangeScreen(true);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], Z_MIN_POS);
            while(!lcd_clicked()) {          
              manage_heater();
              manage_inactivity();
            }

            // PROBE SECOND POINT
            set_ChangeScreen(true);
            set_pageShowInfo(2);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, FRONT_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // PROBE THIRD POINT
            set_ChangeScreen(true);
            set_pageShowInfo(3);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }     

            // PROBE FOURTH POINT
            set_ChangeScreen(true);
            set_pageShowInfo(4);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to(LEFT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // PROBE CENTER
            set_ChangeScreen(true);
            set_pageShowInfo(5);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            do_blocking_move_to((X_MAX_POS-X_MIN_POS)/2, (Y_MAX_POS-Y_MIN_POS)/2, current_position[Z_AXIS]);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS);
            while(!lcd_clicked()) {
              manage_heater();
              manage_inactivity();
            }

            // FINISH MANUAL BED LEVEL
            set_ChangeScreen(true);
            set_pageShowInfo(6);
            do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS],Z_MIN_POS + 5);
            enquecommands_P(PSTR("G28 X0 Y0\nG4 P0\nG4 P0\nG4 P0"));
          #endif // ULTIPANEL
        }
        else if(home_all_axis || homeZ) HOMEAXIS(Z);
      #elif defined(Z_SAFE_HOMING) && defined(ENABLE_AUTO_BED_LEVELING)// Z Safe mode activated.
        if (home_all_axis) {
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
          destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
          feedrate = xy_travel_speed;
          current_position[Z_AXIS] = 0;

          sync_plan_position();
          line_to_destination();
          st_synchronize();
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];
          HOMEAXIS(Z);
        }
        // Let's see if X and Y are homed and probe is inside bed area.
        if (homeZ) {
          if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) {
            float cpx = current_position[X_AXIS], cpy = current_position[Y_AXIS];
            if (   cpx >= X_MIN_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                && cpx <= X_MAX_POS - X_PROBE_OFFSET_FROM_EXTRUDER
                && cpy >= Y_MIN_POS - Y_PROBE_OFFSET_FROM_EXTRUDER
                && cpy <= Y_MAX_POS - Y_PROBE_OFFSET_FROM_EXTRUDER) {
              current_position[Z_AXIS] = 0;
              plan_set_position(cpx, cpy, 0, current_position[E_AXIS]);
              destination[Z_AXIS] = -Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS);    // Set destination away from bed
              feedrate = max_feedrate[Z_AXIS] * 60;
              line_to_destination();
              st_synchronize();
              HOMEAXIS(Z);
            }
            else {
              LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
              SERIAL_ECHO_START;
              SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
            }
          }
          else {
            LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
            SERIAL_ECHO_START;
            SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
          }
        }
      #elif defined(Z_SAFE_HOMING)
        if(home_all_axis || homeZ) {
          destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT);
          destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT);
          feedrate = xy_travel_speed;
          destination[Z_AXIS] = current_position[Z_AXIS] = 0;
          sync_plan_position();
          line_to_destination();
          st_synchronize();
          current_position[X_AXIS] = destination[X_AXIS];
          current_position[Y_AXIS] = destination[Y_AXIS];

          HOMEAXIS(Z);
        }
      #endif //Z_SAFE_HOMING
    #endif //Z_HOME_DIR < 0

    // Set the Z position, if included
    // Adds the home_offset as well, which may be wrong
    if (homeZ) {
      float v = code_value();
      if (v) current_position[Z_AXIS] = v + home_offset[Z_AXIS];
    }

    #ifdef ENABLE_AUTO_BED_LEVELING && (Z_HOME_DIR < 0)
      if (home_all_axis || homeZ) current_position[Z_AXIS] += zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
    #endif //ENABLE_AUTO_BED_LEVELING
    sync_plan_position();
  #endif // defined(CARTESIAN) || defined(COREXY) || defined(SCARA)

  #ifdef SCARA
    sync_plan_position_delta();
  #endif //SCARA
#endif //aven_0807


  #ifdef ENDSTOPS_ONLY_FOR_HOMING
    enable_endstops(false);
  #endif

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  refresh_cmd_timeout();
  endstops_hit_on_purpose(); // clear endstop hit flags
  G28_f = 0; //aven_0807
}
#endif


inline void gcode_X1()
{
  if (code_seen('F')) 
  {
    pleds = code_value_short();
	if(pleds == 1)
    {
      SerialUSB.println("LSA1 OFF");
      digitalWrite(S_LAS1, LOW);
	  
	}
    if(pleds == 2)
    {
      SerialUSB.println("LSA2 OFF");
      digitalWrite(S_LAS2, LOW);
	}
	
    if(pleds == 0)
    {
      SerialUSB.println("LSA1&LSA2 OFF");
      digitalWrite(S_LAS1, LOW);
	  digitalWrite(S_LAS2, LOW);
	}
  }
  
  if (code_seen('O')) 
  {
    pleds = code_value_short();
    if(pleds == 1)
    {
      SerialUSB.println("LSA1 ON");
      digitalWrite(S_LAS1, HIGH);
	}
    if(pleds == 2)
    {
      SerialUSB.println("LSA2 ON");
      digitalWrite(S_LAS2, HIGH);
	}
    if(pleds == 0)
    {
      SerialUSB.println("LSA1&LSA2 ON");
      digitalWrite(S_LAS1, HIGH);
	  digitalWrite(S_LAS2, HIGH);
	}
  }
}

inline void gcode_X2()
{
  if (code_seen('F')) 
  {
    SerialUSB.println("PWM OFF");
    analogWrite(M_IO2, 0);
  }
  
  if (code_seen('O')) 
  {
    pleds = code_value_short();
    if(pleds >255 | pleds <0)
    {
      SerialUSB.print("OUT OF RANGE :");
      SerialUSB.println(pleds);
    }
	else
	{
	  SerialUSB.print("PWM :");
      SerialUSB.println(pleds);
      analogWrite(M_IO2, pleds);
	}	
  }	
}

inline void gcode_X3()
{
  int led_index;
  if(code_seen('L')) {
    led_index = code_value_short();
    SerialUSB.print("MSG LED ");
    SerialUSB.print(led_index);
  } else {
    SerialUSB.println("ER WHERE_IS_CODE_L");
    return;
  }
  if(code_seen('R')) {
    SerialUSB.print(" CONTROL ");
    SerialUSB.print(led_ctrl[led_index]);
    SerialUSB.print(" PARAM_A ");
    SerialUSB.print(led_param_a[led_index]);
    SerialUSB.print(" PARAM_B ");
    SerialUSB.print(led_param_b[led_index]);
    return;
  }
  if(code_seen('C')) {
    led_ctrl[led_index] = code_value();
    SerialUSB.print(" CONTROL ");
    SerialUSB.print(led_ctrl[led_index]);
  }
  if(code_seen('A')) {
    led_param_a[led_index] = code_value();
    SerialUSB.print(" PARAM_A ");
    SerialUSB.print(led_param_a[led_index]);
  }
  if(code_seen('B')) {
    led_param_b[led_index] = code_value();
    SerialUSB.print(" PARAM_B ");
    SerialUSB.print(led_param_b[led_index]);
  }
  SerialUSB.println("#");
}


inline void gcode_X4()
{
  if (code_seen('F')) 
  {
    pleds = code_value_short();
    SerialUSB.println("LED Panel OFF :");
	SerialUSB.println(pleds);
	if(pleds == 1)
    {
      analogWrite(LED_P1, 0);
	}
    if(pleds == 2)
    {
      analogWrite(LED_P2, 0);
	}
	if(pleds == 3)
    {
      analogWrite(LED_P3, 0);
	}
    if(pleds == 0)
    {
      analogWrite(LED_P1, 0);
	  analogWrite(LED_P2, 0);
      analogWrite(LED_P3, 0);
	}
	
  }
  if (code_seen('O')) 
  {
    pleds = code_value_short();
    SerialUSB.println("LED Panel ON :");
	SerialUSB.println(pleds);
    if(pleds == 1)
    {
      analogWrite(LED_P1, 255);
	}
    if(pleds == 2)
    {
      analogWrite(LED_P2, 255);
	}
	if(pleds == 3)
    {
      analogWrite(LED_P3, 255);
	}
    if(pleds == 0)
    {
      analogWrite(LED_P1, 255);
	  analogWrite(LED_P2, 255);
      analogWrite(LED_P3, 255);
	}
  }

}


inline void gcode_X5() //Breathing LED
{
  SerialUSB.println("LED Breathing Setting :");
  if (code_seen('O')) 
  {
    SerialUSB.print("LED 1 Breathing ");
    f_led1=1;
	
    cled1 = code_value_short();
    SerialUSB.print(" count:");
	SerialUSB.print(cled1);

	if(code_seen('C'))
	{
      fled1 = code_value_short();
	  SerialUSB.print(" freq:");
	  SerialUSB.println(fled1);
	}

	
    leds1=1;
	
  }	



  
  if (code_seen('P')) 
  {
      SerialUSB.print("LED 2 Breathing ");
      f_led2=1;
	  cled2 = code_value_short();
	  SerialUSB.print(" count:");
	  SerialUSB.print(cled2);

	  if(code_seen('C'))
	  {
        fled2 = code_value_short();
		SerialUSB.print(" freq:");
	    SerialUSB.println(fled2);
	  }

      leds1=1;
	  
  } 

  
  if (code_seen('Q')) 
  {
	  SerialUSB.print("LED 3 Breathing ");
	  f_led3=1;
	  cled3 = code_value_short();
	  SerialUSB.print(" count:");
	  SerialUSB.print(cled3);

	  if(code_seen('C'))
	  {
        fled3 = code_value_short();
		SerialUSB.print(" freq:");
	    SerialUSB.println(fled3);
	  }

	 
        leds1=1;
	
  } 

  
  if (code_seen('R')) 
  {
	  SerialUSB.print("ALL LED Breathing ");
	  //SerialUSB.print("LED 2 Breathing ");
	  //SerialUSB.print("LED 3 Breathing ");

	  f_led1 = 1;
	  f_led2 = 1;
	  f_led3 = 1;
	  
	  cled1 = code_value_short();
	  cled2=cled1;
	  cled3=cled2;

	  SerialUSB.print(" count:");
	  SerialUSB.print(cled3);

	  if(code_seen('C'))
	  {
        fled1 = code_value_short();
		fled2=fled1;
		fled3=fled2;

		SerialUSB.print(" freq:");
	    SerialUSB.println(fled3);
		
	  }

	  
        leds1=1;
		leds2=1;
		leds3=1;
	  
	} 

   if (code_seen('F')) 
   {
     SerialUSB.print("LED Breathing off");
	 fled1=0;
	 fled2=0;
	 fled3=0;

	 cled1=0;
	 cled2=0;
	 cled3=0;

	 fled1=0;
	 fled2=0;
	 fled3=0;

	 leds1=0;
	 leds2=0;
	 leds3=0;

	 dcount1=0;
	 dcount2=0;
	 dcount3=0;

	 runleds1=0;
	 runleds2=0;
	 runleds3=0;

     
	 delay(100);
	 analogWrite(LED_P2,0);
	 delay(100);
	 analogWrite(LED_P3,0);
	 delay(100);
	 analogWrite(LED_P1,255);
	 //delay(100);

	 
   }
  
}


#if 0
inline void gcode_X5() //heater PID
{
  setpoint = 0;
  //power = 0;
  
  if (code_seen('H') || code_seen('h')) 
  {
    //SerialUSB.print("Setpoint 1 : ");
    //SerialUSB.println(setpoint);
    //setpoint = constrain(code_value_short(), 0, 2625);
    setpoint =code_value_short();

    //SerialUSB.print("Setpoint 2 : ");
    //SerialUSB.println(setpoint);
	if(setpoint >= 0 || setpoint <= 2625)
    {
    //pbuf[0]= 'h';
	//pbuf[1]= code_value_short();
	
     SerialUSB.println("Heater ON ");
	
	 Serial.print("h");
	 Serial.print(setpoint);
	}
	else
	{
      SerialUSB.print("Wrong Temp !!");
	  SerialUSB.print("Range 0 ~ 2625");
	}
  }
  

  
  if (code_seen('O') | code_seen('o')) 
  {
    SerialUSB.println("Heater OFF");
	Serial.print("O");
  }



/*
  if (code_seen('X') | code_seen('x')) 
  {
    SerialUSB.print("power 1 : ");
    SerialUSB.println(power);
    
    power =code_value_short();

	SerialUSB.print("power 2 : ");
    SerialUSB.println(power);

    SerialUSB.print("Set Laser power ");
	
	if(power>256)
	{
      SerialUSB.print("Wrong power !!");
	  SerialUSB.print("Range 0 ~ 255");
	}
    else
    {
	  Serial.print("L");
	  Serial.print(power);
	}
  }*/

  

}
#endif


#if 0
inline void gcode_X6() //Read  Temp
{
    read_temp = 0;
  //if (code_seen('T') | code_seen('t')) 
  //{
    SerialUSB.println("Read Temp :");
    Serial.print("t");


    int byteCount = Serial.readBytesUntil(',', RX_buff, 50);
    if(byteCount > 0)
    {
      for(int i = 0; i < byteCount; i++)
      {  
        SerialUSB.write(RX_buff[i]);
               
      }
    }
	//free(RX_buff);
}


inline void gcode_X7() //set frequency
{
  freq = 0;
  if (code_seen('P') | code_seen('p')) 
  {
    SerialUSB.println("Set Fan2 frequency ");
    freq =code_value_short();
	if(freq == 1 | freq == 8 | freq == 64 | freq == 256 | freq == 1024)
	{
      Serial.print("p");
	  Serial.print(freq);
	}
    else
    {
      SerialUSB.println("Wrong freq !!");
	  SerialUSB.println("1 or 8 or 64 or 256 or 1024");
	}	
  }
}



inline void gcode_X8() //set fan duty
{
  duty = 0;

  if(code_seen('F'))
  {
    duty = code_value();
    SerialUSB.println("Set Fan1 & Fan2 duty ");
    SerialUSB.println(duty);
	Serial.print("F");
	Serial.print(duty);
  }	
}


//Only for Laser Module
inline void gcode_X9() //set Laser power
{
  power = 0;
  
  
  if (code_seen('L')) 
  {
    power =code_value_short();
	SerialUSB.println("Set Laser power ");
    SerialUSB.println(power);   
	Serial.print("L");
	Serial.print(power);	
  }
}


inline void gcode_X10() //Read status
{
    SerialUSB.println("Read Status ");
	Serial.print("R");

    int byteCount = Serial.readBytesUntil(',', RX_buff, 100);
    if(byteCount > 0)
    {
      for(int i = 0; i < byteCount; i++)
      {  
        SerialUSB.write(RX_buff[i]) ;             
      }
    }
	//free(RX_buff);
	//delay(100);
}

#endif


inline void gcode_X11()
{
       gcode_G28();
	   delay(100);
       float x = 0;
       float y = 0;
	   float h_value = 0;
  
	   deploy_z_probe();
	   h_value = probe_bed(x,y);
	   SERIAL_ECHO(" h_value : ");
	   SERIAL_PROTOCOL_F(h_value, 4);
	   SERIAL_EOL;
  
	   retract_z_probe();

	   max_pos[Z_AXIS]= max_pos[Z_AXIS] - h_value;
       set_delta_constants();
       SERIAL_ECHO(" H : ");
	   SERIAL_PROTOCOL_F(max_pos[Z_AXIS], 4);

	   h_offset = h_value;
	   delay(100);
       gcode_G28();
	   delay(100);
	   
	         	      
}


inline void gcode_X12(float kvalue)
{
  int coo =0;
  gcode_G28();
  delay(100);


  SERIAL_ECHO(" X12 : ");
  SERIAL_ECHO(" kvalue : ");
  SERIAL_PROTOCOL_F(kvalue, 4);

  while(1)
  {
   float x = -73.61;
   float y = -42.5;
   float x_value = 0;
   deploy_z_probe();
   x_value = probe_bed(x,y);
   SERIAL_ECHO(" x_value : ");
   SERIAL_PROTOCOL_F(x_value, 4);
   SERIAL_EOL;

   retract_z_probe();

       gcode_G28();
	   delay(100);

	   x = 73.61;
	   y = -42.5;
	   float y_value = 0;
	   float vv =0;
       deploy_z_probe();
	   y_value = probe_bed(x,y);
	   SERIAL_ECHO(" y_value : ");
       SERIAL_PROTOCOL_F(y_value, 4);
	   SERIAL_EOL;
	   retract_z_probe();
	   coo++;
       SERIAL_ECHO(" count : ");
       SERIAL_PROTOCOL_F(coo, 4);
  
	delay(100);

	if(x_value > y_value)
	{
 	  if(x_value - y_value < kvalue)
      {
               SERIAL_ECHO(" x_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[0], 4);
		       SERIAL_ECHO(" y_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[1], 4);
			   gcode_G28();
			   delay(100);
		       break;
      }
	  else
      {
               vv = ((y_value - x_value)*0.8);
	           endstop_adj[1] = endstop_adj[1] + vv;
               set_delta_constants();
	           SERIAL_ECHO(" vv : ");
               SERIAL_PROTOCOL_F(endstop_adj[1], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
	  }
	}  
    else
    {
      if(y_value - x_value < kvalue)
      {
               SERIAL_ECHO(" x_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[0], 4);
		       SERIAL_ECHO(" y_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[1], 4);
			   gcode_G28();
			   delay(100);
		       break;
      }
	  else
      {
               vv = ((x_value - y_value)*0.8);
	           endstop_adj[0] = endstop_adj[0] + vv;
               set_delta_constants();
	           SERIAL_ECHO(" vv : ");
               SERIAL_PROTOCOL_F(endstop_adj[0], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
	  }

    }
    	
	  
  }   

}

inline void gcode_X13(float kvalue)
{
  int coo =0;
  gcode_G28();
  delay(100);

  SERIAL_ECHO(" X13 : ");
  SERIAL_ECHO(" kvalue : ");
  SERIAL_PROTOCOL_F(kvalue, 4);

  while(1)
  {
   float x = 73.61;
   float y = -42.5;
   float y_value = 0;
   deploy_z_probe();
   y_value = probe_bed(x,y);
   SERIAL_ECHO(" y_value : ");
   SERIAL_PROTOCOL_F(y_value, 4);
   SERIAL_EOL;

   retract_z_probe();

       gcode_G28();
	   delay(100);

	   x = 0;
	   y = 85;
	   float z_value = 0;
	   float vv =0;
       deploy_z_probe();
	   z_value = probe_bed(x,y);
	   SERIAL_ECHO(" z_value : ");
       SERIAL_PROTOCOL_F(z_value, 4);
	   SERIAL_EOL;
	   retract_z_probe();
	   coo++;
       SERIAL_ECHO(" count : ");
       SERIAL_PROTOCOL_F(coo, 4);
  
	delay(100);

	if(y_value > z_value)
	{
      if(y_value - z_value < kvalue)
      {
               SERIAL_ECHO(" y_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[1], 4);
		       SERIAL_ECHO(" z_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[2], 4);
			   gcode_G28();
			   delay(100);
		       break;
      }
	  else
      {
               vv = ((z_value - y_value)*0.8);
	           endstop_adj[2] = endstop_adj[2] + vv;
               set_delta_constants();
	           SERIAL_ECHO(" vv : ");
               SERIAL_PROTOCOL_F(endstop_adj[2], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
	  }
	} 
	else
	{
      if(z_value - y_value < kvalue)
      {
               SERIAL_ECHO(" y_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[1], 4);
		       SERIAL_ECHO(" z_end : ");
               SERIAL_PROTOCOL_F(endstop_adj[2], 4);
			   gcode_G28();
			   delay(100);
		       break;
      }
	  else
      {
               vv = ((y_value - z_value)*0.8);
	           endstop_adj[1] = endstop_adj[1] + vv;
               set_delta_constants();
	           SERIAL_ECHO(" vv : ");
               SERIAL_PROTOCOL_F(endstop_adj[1], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
	  }


	}
  }   

}

inline void gcode_X14(float kvalue)
{
   
   int coo =0;
	 gcode_G28();
	 delay(100);
   
	 while(1)
	 {
	  float x = 0;
	  float y = 0;
	  float r_value = 0;
	  deploy_z_probe();
	  r_value = probe_bed(x,y);
	  SERIAL_ECHO(" r_value : ");
	  SERIAL_PROTOCOL_F(r_value, 4);
	  SERIAL_EOL;
   
	  retract_z_probe();
   
	  gcode_G28();
	  delay(100);
   
		  x = -73.61;
		  y = -42.5;
		  float r1_value = 0;
		  float vv =0;
		  deploy_z_probe();
		  r1_value = probe_bed(x,y);
		  SERIAL_ECHO(" r1_value : ");
		  SERIAL_PROTOCOL_F(r1_value, 4);
		  SERIAL_EOL;
		  retract_z_probe();
		  coo++;
		  SERIAL_ECHO(" count : ");
		  SERIAL_PROTOCOL_F(coo, 4);
	 
	   delay(100);
	   if(r_value > r1_value)
	   {
	     if(r1_value - r_value < kvalue)
	     {
				  SERIAL_ECHO(" delta_radius : ");
				  SERIAL_PROTOCOL_F(delta_radius, 4);
				  gcode_G28();
				  delay(100);
				  break;
	     }
	     else
	     {
		          vv = ((r1_value - r_value)*0.8);
		          delta_radius = delta_radius + vv;
		          set_delta_constants();
		          SERIAL_ECHO(" vv : ");
		          SERIAL_PROTOCOL_F(delta_radius, 4);
		          delay(100);
		          gcode_G28();
		          delay(100);
	     }
	   }
	   else
	   {
         if(r_value - r1_value < kvalue)
	     {
				  SERIAL_ECHO(" delta_radius : ");
				  SERIAL_PROTOCOL_F(delta_radius, 4);
				  gcode_G28();
				  delay(100);
				  break;
	     }
	     else
	     {
		          vv = ((r_value - r1_value)*0.8);
		          delta_radius = delta_radius + vv;
		          set_delta_constants();
		          SERIAL_ECHO(" vv : ");
		          SERIAL_PROTOCOL_F(delta_radius, 4);
		          delay(100);
		          gcode_G28();
		          delay(100);
	     }


	   }
	 }	 
}

inline void gcode_X15()
{
  if(code_seen('K'))
  {
    u_value =code_value();

    SERIAL_ECHO(" u_value : ");
    SERIAL_PROTOCOL_F(u_value, 4);
  
    gcode_X11();
    gcode_X12(u_value);
    gcode_X13(u_value);
	gcode_X14(u_value);
  }

  else
  {
    gcode_X11();
    gcode_X12(k_value);
    gcode_X13(k_value);
	gcode_X14(k_value);
  }

  float x = 0;
  float y = 0;
  float h_value = 0;
  deploy_z_probe();
  h_value = probe_bed(x,y);
  SERIAL_ECHO(" h_value : ");
  SERIAL_PROTOCOL_F(h_value, 4);
  SERIAL_EOL; 
  retract_z_probe();
  delay(100);
  gcode_G28();
  delay(100);

  x = -73.61;
  y = -42.5;
  float x_value = 0;
  deploy_z_probe();
  x_value = probe_bed(x,y);
  SERIAL_ECHO(" x_value : ");
  SERIAL_PROTOCOL_F(x_value, 4);
  SERIAL_EOL; 
  retract_z_probe();
  delay(100);
  gcode_G28();
  delay(100);

  x = 73.61;
  y = -42.5;
  float y_value = 0;
  deploy_z_probe();
  y_value = probe_bed(x,y);
  SERIAL_ECHO(" y_value : ");
  SERIAL_PROTOCOL_F(y_value, 4);
  SERIAL_EOL;
  retract_z_probe();
  delay(100);
  gcode_G28();
  delay(100);

  x = 0;
  y = 85;
  float z_value = 0;
  float vv =0;
  deploy_z_probe();
  z_value = probe_bed(x,y);
  SERIAL_ECHO(" z_value : ");
  SERIAL_PROTOCOL_F(z_value, 4);
  SERIAL_EOL;
  retract_z_probe();
  delay(100);
  gcode_G28();
  delay(100);	   
  
}

inline void gcode_C1()
{
  if (code_seen('O'))
  {
    SERIAL_PROTOCOLLN("CTRL LINECHECK_ENABLED");
    line_check = 1;
    gcode_LastN = 0;
  } else if (code_seen('F')) {
    SERIAL_PROTOCOLLN("CTRL LINECHECK_DISABLED");
    line_check = 0;
  } else {
    SERIAL_PROTOCOLLN("ER BAD_CMD");
  }
}

#if 0

inline void gcode_X16()
{
  if (code_seen('S')) 
  {
    int line_num = code_value();
	SerialUSB.println(line_num);
	NO = line_num;
  }
  
  if (code_seen('R')) 
  {
    SerialUSB.println(NO);
  }

}

inline void gcode_X18()
{
    SerialUSB.print("FLUX Main Board Version : ");
	SerialUSB.println(FW_V);	

}

//aven_0813
inline void gcode_X19()
{
    //delta[X_AXIS]=180.04;
	//delta[Y_AXIS]=190.87;
	//delta[Z_AXIS]=203.30;
    //actuator_to_cartesian(delta);
    if (code_seen('A')) {
      delta[X_AXIS] = code_value();
	  SerialUSB.println(delta[X_AXIS]);
    }

	if (code_seen('B')) {
      delta[Y_AXIS] = code_value();
	  SerialUSB.println(delta[Y_AXIS]);
    }

	if (code_seen('C')) {
      delta[Z_AXIS] = code_value();
	  SerialUSB.println(delta[Z_AXIS]);
    }
	
	actuator_to_cartesian(delta);
	
#if 0
  SerialUSB.print("Get Position X: ");
  SerialUSB.print(st_get_position(X_AXIS));
  SerialUSB.println(float(st_get_position(X_AXIS)));
  SerialUSB.print(" Y : ");
  SerialUSB.print(st_get_position(Y_AXIS));
  SerialUSB.print(" Z : ");
  SerialUSB.println(st_get_position(Z_AXIS));
#endif
  
#if 0
  
  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  refresh_cmd_timeout();

  enable_endstops(true);

  set_destination_to_current();

  feedrate = 0.0;

  bool  homeX = code_seen(axis_codes[X_AXIS]),
        homeY = code_seen(axis_codes[Y_AXIS]),
        homeZ = code_seen(axis_codes[Z_AXIS]),
        homeE = code_seen(axis_codes[E_AXIS]);


  home_all_axis = !(homeX || homeY || homeZ || homeE || home_x || home_y) || (homeX && homeY && homeZ);

  for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
    sync_plan_position();//X:0.00
	
    // Move all carriages up together until the first endstop is hit.
    for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
    feedrate = 1.732 * homing_feedrate[X_AXIS];
    line_to_destination();
	
    st_synchronize();
	
    endstops_hit_on_purpose(); // clear endstop hit flags

   


#endif


#if 0
 do{ 

    if(READ(A_STOP)==1)
    {  
      SerialUSB.println("A STOP!");
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SerialUSB.println(" Count X: ");
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
	  ASTOP = true;
    }
 
    if(READ(B_STOP)==1)
    {
      SerialUSB.println("B STOP!");
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SerialUSB.println(" Count Y: ");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
	  BSTOP = true;
    }
 
    if(READ(C_STOP)==1)
    {
      SerialUSB.println("C STOP!");
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SerialUSB.println(" Count Z: ");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
	  CSTOP = true;
    }
 	}
	while((ASTOP = true) and (BSTOP = true) and (CSTOP = true));
#endif


#if 0
 HOMEAXIS(X);
 HOMEAXIS(Y);
 HOMEAXIS(Z);
 enable_endstops(false);

 feedrate = saved_feedrate;
 feedmultiply = saved_feedmultiply;
 refresh_cmd_timeout(); 
 endstops_hit_on_purpose(); // clear endstop hit flags
#endif  
  
 
#if 0
 if(READ(Z_PROBE_PIN)==1)
 {
   SerialUSB.println("Z probe STOP!");
 }
#endif


}



inline void gcode_X20(boolean home_x = false, boolean home_y = false)
{

  gcode_G28();

  delay(1000);
  
  odelta[0] = st_get_position(X_AXIS)/axis_steps_per_unit[X_AXIS];
  odelta[1] = st_get_position(Y_AXIS)/axis_steps_per_unit[Y_AXIS];
  odelta[2] = st_get_position(Z_AXIS)/axis_steps_per_unit[Z_AXIS];
  
  SerialUSB.print("Default X: ");
  SerialUSB.print(odelta[0]);
  SerialUSB.print(" Y: ");
  SerialUSB.print(odelta[1]);
  SerialUSB.print(" Z: ");
  SerialUSB.println(odelta[2]);

  delay(500);
  
  disable_x();
  disable_y();
  disable_z();

  SerialUSB.println("Done!");

  

}


inline void gcode_X21(boolean home_x = false, boolean home_y = false)
{

#if 1
  
  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;
  refresh_cmd_timeout();

  enable_endstops(true);

  set_destination_to_current();

  feedrate = 0.0;

  bool  homeX = code_seen(axis_codes[X_AXIS]),
        homeY = code_seen(axis_codes[Y_AXIS]),
        homeZ = code_seen(axis_codes[Z_AXIS]),
        homeE = code_seen(axis_codes[E_AXIS]);


  home_all_axis = !(homeX || homeY || homeZ || homeE || home_x || home_y) || (homeX && homeY && homeZ);

#if 0
  sync_plan_position();
  destination[X_AXIS] = 200;
  destination[Y_AXIS] = 200;
  destination[Z_AXIS] = 200;
  feedrate = 1.732 * homing_feedrate[X_AXIS];
  line_to_destination();
  st_synchronize();
  //plan_buffer_line();
#endif

  #if 1
  for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
	  sync_plan_position();
  
	  // Move all carriages up together until the first endstop is hit.
	  //for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = 3 * Z_MAX_LENGTH;
	  for (int i = X_AXIS; i <= Z_AXIS; i++) {destination[i] = 470;}
	  feedrate = 1.732 * homing_feedrate[X_AXIS];
	  line_to_destination();
	  st_synchronize();
	  endstops_hit_on_purpose(); // clear endstop hit flags
  
	  // Destination reached
	  for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = destination[i];
  
      endstops_hit_on_purpose(); // clear endstop hit flags


	  cdelta[0] = st_get_position(X_AXIS)/axis_steps_per_unit[X_AXIS];
      cdelta[1] = st_get_position(Y_AXIS)/axis_steps_per_unit[Y_AXIS];
      cdelta[2] = st_get_position(Z_AXIS)/axis_steps_per_unit[Z_AXIS];

	  SerialUSB.print("Default X: ");
      SerialUSB.print(cdelta[0]);
      SerialUSB.print(" Y: ");
      SerialUSB.print(cdelta[1]);
      SerialUSB.print(" Z: ");
      SerialUSB.println(cdelta[2]);

  delay(500);
  #endif
   


#endif


#if 0
 do{ 

    if(READ(A_STOP)==1)
    {  
      SerialUSB.println("A STOP!");
      SERIAL_PROTOCOLPGM("X:");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SerialUSB.println(" Count X: ");
      SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
	  ASTOP = true;
    }
 
    if(READ(B_STOP)==1)
    {
      SerialUSB.println("B STOP!");
      SERIAL_PROTOCOLPGM("Y:");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SerialUSB.println(" Count Y: ");
      SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
	  BSTOP = true;
    }
 
    if(READ(C_STOP)==1)
    {
      SerialUSB.println("C STOP!");
      SERIAL_PROTOCOLPGM("Z:");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SerialUSB.println(" Count Z: ");
      SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
	  CSTOP = true;
    }
 	}
	while((ASTOP = true) and (BSTOP = true) and (CSTOP = true));
#endif


#if 0
 HOMEAXIS(X);
 SerialUSB.println(delta[X_AXIS]);

 HOMEAXIS(Y);
 SerialUSB.println(delta[X_AXIS]);
 
 HOMEAXIS(Z);
 SerialUSB.println(delta[X_AXIS]);
 //sync_plan_position_delta();

 
 enable_endstops(false);
 SerialUSB.println(delta[X_AXIS]);

 feedrate = saved_feedrate;
 feedmultiply = saved_feedmultiply;
 refresh_cmd_timeout(); 
 endstops_hit_on_purpose(); // clear endstop hit flags
#endif  
  
 
#if 0
 if(READ(Z_PROBE_PIN)==1)
 {
   SerialUSB.println("Z probe STOP!");
 }
#endif


}

#endif

inline void gcode_X6()
{

  boolean home_x = false; 
  boolean home_y = false;
  odelta[0]=0;
  odelta[1]=0;
  odelta[2]=0;
  cdelta[0]= 0;
  cdelta[1]= 0;
  cdelta[2]= 0;

  
#if 1
  float h_ori[3]={0,0,0};
  h_ori[2]= max_length[Z_AXIS];
  float h_cal[3];
  calculate_delta_position(h_ori,h_cal,delta_radius);
  float h_constant = h_cal[0];
#endif  

//aven_0817
  //for(int i=0 ; i<600 ; i++)
  for(int i=0 ; i<3 ; i++)
  {
    saved_feedrate = feedrate;
	saved_feedmultiply = feedmultiply;
	feedmultiply = 100;
	refresh_cmd_timeout();
  
	enable_endstops(true);
  
	set_destination_to_current();
  
	feedrate = 0.0;
  
	bool  homeX = code_seen(axis_codes[X_AXIS]),
		  homeY = code_seen(axis_codes[Y_AXIS]),
		  homeZ = code_seen(axis_codes[Z_AXIS]),
		  homeE = code_seen(axis_codes[E_AXIS]);
  
  
	home_all_axis = !(homeX || homeY || homeZ || homeE || home_x || home_y) || (homeX && homeY && homeZ);
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
	  sync_plan_position();
	  
    for (int i = X_AXIS; i <= Z_AXIS; i++) 
	{
//aven_0817	
	  //destination[i] = 1;
	  destination[i] = 600;

#if 1	  
      if(READ(A_STOP)==1)
      {  
        //SerialUSB.println("A STOP!");
        //disable_x();
		destination[0] = 0;	  
      }
 
      if(READ(B_STOP)==1)
      {
        //SerialUSB.println("B STOP!");
        //disable_y();
		destination[1] = 0;
      }
 
      if(READ(C_STOP)==1)
      {
        //SerialUSB.println("C STOP!");
        //disable_e();
		destination[2] = 0;
      }
#endif
      check_axes_activity();


	 }
//aven_0817	
	  //feedrate = 1.732 * homing_feedrate[X_AXIS];
	  feedrate = 1 * homing_feedrate[X_AXIS];

	  line_to_destination();
	  st_synchronize();

       odelta[0] = st_get_position(X_AXIS)/axis_steps_per_unit[X_AXIS];
       odelta[1] = st_get_position(Y_AXIS)/axis_steps_per_unit[Y_AXIS];
       odelta[2] = st_get_position(Z_AXIS)/axis_steps_per_unit[Z_AXIS];

       

	   //odelta[0] = st_get_position(X_AXIS);
	   //odelta[1] = st_get_position(Y_AXIS);
	   //odelta[2] = st_get_position(Z_AXIS);
      
	  cdelta[0] = cdelta[0] + odelta[0];
      cdelta[1] = cdelta[1] + odelta[1];
      cdelta[2] = cdelta[2] + odelta[2];

      #if 0
      SerialUSB.print(i);
	  SerialUSB.print(" X: ");
      SerialUSB.print(cdelta[0]);
      SerialUSB.print(" Y: ");
      SerialUSB.print(cdelta[1]);
      SerialUSB.print(" Z: ");
      SerialUSB.println(cdelta[2]);
      #endif

	  
	  

  	}

    //cdelta[0] = (cdelta[0]/80) - 403.23;
	//cdelta[1] = (cdelta[1]/80) - 403.23;
	//cdelta[2] = (cdelta[2]/80) - 403.23;

	enable_endstops(false);

	cdelta[0] = h_constant - cdelta[0];
	cdelta[1] = h_constant - cdelta[1];
	cdelta[2] = h_constant - cdelta[2];

	SerialUSB.print("h_constant: ");
	SerialUSB.println(h_constant);

    //cdelta[0] = 403.23 - cdelta[0];
	//cdelta[1] = 403.23 - cdelta[1];
	//cdelta[2] = 403.23 - cdelta[2];

    //SerialUSB.print(i);
	//SerialUSB.print("X: ");
    //SerialUSB.print(cdelta[0]);
    //SerialUSB.print(" Y: ");
    //SerialUSB.print(cdelta[1]);
    //SerialUSB.print(" Z: ");
    //SerialUSB.println(cdelta[2]);

	//gcode_X19(cdelta);

	actuator_to_cartesian(cdelta);

    SerialUSB.println("@X22:");
	SerialUSB.print("X:");
	SerialUSB.print(cartesian[X_AXIS]);
	SerialUSB.print(" Y:");
	SerialUSB.print(cartesian[Y_AXIS]);
	SerialUSB.print(" Z:");
	SerialUSB.print(cartesian[Z_AXIS]);

	SerialUSB.print(" Count X:");
	SerialUSB.print(cdelta[X_AXIS]);
	SerialUSB.print(" Y:");
	SerialUSB.print(cdelta[Y_AXIS]);
	SerialUSB.print(" Z:");
	SerialUSB.println(cdelta[Z_AXIS]);

    delay(100);
	gcode_G28();
	delay(500);

	

}

void step(int num,boolean dir,int steps)
{
	  
	  if(num == 1)
	  {
	  
		  digitalWrite(EN1,LOW);
		  delay(100);
		  SerialUSB.print("Motor :");
		  SerialUSB.print(num);
		  SerialUSB.print("  steps :");
		  SerialUSB.print(steps);
		  
		  digitalWrite(DIR1,dir);
		  delay(50);
		  for(int i=0;i<steps;i++)
		  {
			  digitalWrite(STP1, HIGH);
			  delayMicroseconds(800);
			  digitalWrite(STP1, LOW);
			  delayMicroseconds(800);
		   }
		   digitalWrite(EN1,HIGH);
	  }
	  
	  if(num == 2)
	  {
	  
		  digitalWrite(EN2,LOW);
		  delay(100);
		  SerialUSB.print("Motor :");
		  SerialUSB.print(num);
		  SerialUSB.print("  steps :");
		  SerialUSB.print(steps);
		  
		  digitalWrite(DIR2,dir);
		  delay(50);
		  for(int i=0;i<steps;i++)
		  {
			  digitalWrite(STP2, HIGH);
			  delayMicroseconds(800);
			  digitalWrite(STP2, LOW);
			  delayMicroseconds(800);
		   }
		   digitalWrite(EN2,HIGH);
	  } 
	
	  if(num == 3)
	  {
	  
		  digitalWrite(EN3,LOW);
		  delay(100);
		  SerialUSB.print("Motor :");
		  SerialUSB.print(num);
		  SerialUSB.print("  steps :");
		  SerialUSB.print(steps);
		  
		  digitalWrite(DIR3,dir);
		  delay(50);
		  for(int i=0;i<steps;i++)
		  {
			  digitalWrite(STP3, HIGH);
			  delayMicroseconds(800);
			  digitalWrite(STP3, LOW);
			  delayMicroseconds(800);
		   }
		   digitalWrite(EN3,HIGH);
	  } 
	
	  if(num == 4)
	  {
	  
		  digitalWrite(EN4,LOW);
		  delay(100);
		  SerialUSB.print("Motor :");
		  SerialUSB.print(num);
		  SerialUSB.print("  steps :");
		  SerialUSB.print(steps);
		  
		  digitalWrite(DIR4,dir);
		  delay(50);
		  for(int i=0;i<steps;i++)
		  {
			  digitalWrite(STP4, HIGH);
			  delayMicroseconds(800);
			  digitalWrite(STP4, LOW);
			  delayMicroseconds(800);
		   }
		   digitalWrite(EN4,HIGH);
	  } 
	   
	
	
	  if(num == 5)
	  {
	
		digitalWrite(EN5,LOW);
		delay(100);
		SerialUSB.print("Motor :");
		SerialUSB.print(num);
		SerialUSB.print("  steps :");
		SerialUSB.print(steps);
		
		digitalWrite(DIR5,dir);
		delay(50);
		for(int i=0;i<steps;i++)
		{
			digitalWrite(STP5, HIGH);
			delayMicroseconds(800);
			digitalWrite(STP5, LOW);
			delayMicroseconds(800);
		 }
		 digitalWrite(EN5,HIGH);
	  } 
	
	
	
	  if(num == 6)
	  {
	
		digitalWrite(EN6,LOW);
		delay(100);
		SerialUSB.print("Motor :");
		SerialUSB.print(num);
		SerialUSB.print("  steps :");
		SerialUSB.print(steps);
		
		digitalWrite(DIR6,dir);
		delay(50);
		for(int i=0;i<steps;i++)
		{
			digitalWrite(STP6, HIGH);
			delayMicroseconds(800);
			digitalWrite(STP6, LOW);
			delayMicroseconds(800);
		 }
		 digitalWrite(EN6,HIGH);
	  }
}



inline void gcode_X7()
{
  if (code_seen('R')) 
  {
    motors = code_value_short();

	if (code_seen('C')) 
    {
      motorc = code_value_short();
	}
	
    step(motors,true,motorc);
    delay(500);
    step(motors,false,motorc*5);
    delay(500);
	
  }
}


#if 0
inline void gcode_X23()
{
  bool r_en = true; // enable R modification
  bool h_en = true; // enable H modification

	/* 4 or 3 points input. If r_en == false && h_en == false, only 3 points needed.
	float p[4][3] = 
	{
		{x0, y0, z0},  	1st point should near {-73.61, -42.5, z0}
		{x1, y1, z1},	2nd point should near {73.61 , -42.5, z1}
		{x2, y2, z2},   3rd point should near {0	 , 0	, z2}
		{x3, y3, z3}    4th point should near the center
	};
	*/
	float p[4][3];


	float error[5];
	for (int i = 0; i < 3; i++) error[i] = -1 * endstop_adj[i];
	error[3] = delta_radius;
	error[4] = max_pos[Z_AXIS];

	if(calculate_error(p, error, r_en, h_en))
	{
		//cout << "OK" << endl;
		for (int i = 0; i < 3; i++) endstop_adj[i] = -1 * error[i];
		if (r_en) delta_radius = error[3];
		if (h_en) max_pos[Z_AXIS] = error[4];
		set_delta_constants();
	}

	
	//return 0;
}


#if 0
inline void gcode_X23()
{
  boolean home_x = false;
  boolean home_y = false;
  float h_value=0; 
  odelta[0]=0;
  odelta[1]=0;
  odelta[2]=0;
  cdelta[0]= 0;
  cdelta[1]= 0;
  cdelta[2]= 0;
  
  gcode_G28();

  delay(100);


  
  
  for(int i=245 ; i>0 ; i--)
  {
    saved_feedrate = feedrate;
	saved_feedmultiply = feedmultiply;
	feedmultiply = 100;
	refresh_cmd_timeout();
  
	enable_endstops(true);
  
	set_destination_to_current();
  
	feedrate = 0.0;
  
	bool  homeX = code_seen(axis_codes[X_AXIS]),
		  homeY = code_seen(axis_codes[Y_AXIS]),
		  homeZ = code_seen(axis_codes[Z_AXIS]),
		  homeE = code_seen(axis_codes[E_AXIS]);
  
  
	home_all_axis = !(homeX || homeY || homeZ || homeE || home_x || home_y) || (homeX && homeY && homeZ);
    for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
	  sync_plan_position();
	  
    for (int i = X_AXIS; i <= Z_AXIS; i++) 
	{
	  destination[i] = -1;

#if 0	  
      if(READ(A_STOP)==1)
      {  
        //SerialUSB.println("A STOP!");
        //disable_x();
		destination[0] = 0;	  
      }
 
      if(READ(B_STOP)==1)
      {
        //SerialUSB.println("B STOP!");
        //disable_y();
		destination[1] = 0;
      }
 
      if(READ(C_STOP)==1)
      {
        //SerialUSB.println("C STOP!");
        //disable_e();
		destination[2] = 0;
      }
#endif
      check_axes_activity();


	 }
	
	  feedrate = 1.732 * homing_feedrate[X_AXIS];
	  line_to_destination();
	  st_synchronize();

       odelta[0] = st_get_position(X_AXIS)/axis_steps_per_unit[X_AXIS];
       odelta[1] = st_get_position(Y_AXIS)/axis_steps_per_unit[Y_AXIS];
       odelta[2] = st_get_position(Z_AXIS)/axis_steps_per_unit[Z_AXIS];

       

	   //odelta[0] = st_get_position(X_AXIS);
	   //odelta[1] = st_get_position(Y_AXIS);
	   //odelta[2] = st_get_position(Z_AXIS);
      
	  cdelta[0] = cdelta[0] + odelta[0];
      cdelta[1] = cdelta[1] + odelta[1];
      cdelta[2] = cdelta[2] + odelta[2];

      //SerialUSB.print(i);
	  //SerialUSB.print(" X: ");
      //SerialUSB.print(cdelta[0]);
      //SerialUSB.print(" Y: ");
      //SerialUSB.print(cdelta[1]);
      //SerialUSB.print(" Z: ");
      //SerialUSB.println(cdelta[2]);


	  
	  

  	}

    //cdelta[0] = (cdelta[0]/80) - 403.23;
	//cdelta[1] = (cdelta[1]/80) - 403.23;
	//cdelta[2] = (cdelta[2]/80) - 403.23;

	cdelta[0] = cdelta[0] + (403.23-h_offset);
	cdelta[1] = cdelta[1] + (403.23-h_offset);
	cdelta[2] = cdelta[2] + (403.23-h_offset);

    //cdelta[0] = 403.23 - cdelta[0];
	//cdelta[1] = 403.23 - cdelta[1];
	//cdelta[2] = 403.23 - cdelta[2];

    //SerialUSB.print(i);
	//SerialUSB.print(" X: ");
    //SerialUSB.print(cdelta[0]);
    //SerialUSB.print(" Y: ");
    //SerialUSB.print(cdelta[1]);
    //SerialUSB.print(" Z: ");
    //SerialUSB.println(cdelta[2]);

	//gcode_X19(cdelta);

	actuator_to_cartesian(cdelta);

	delay(500);

	gcode_X22();

	h_value = cartesian[Z_AXIS];
	

	max_pos[Z_AXIS]= max_pos[Z_AXIS] - h_value;
    set_delta_constants();
	SerialUSB.println("");
    SerialUSB.print(" H : ");
	SerialUSB.println(max_pos[Z_AXIS], 4);

	delay(500);

	gcode_G28();

	delay(500);

	
	

}
#endif

inline void gcode_X24()
{
  float vv=0;
  //float v1_offset=0;
  //float v2_offset=0;
  float v_value=0;

  //gcode_X23();

  delay(500);
  
  gcode_G28();
  delay(100);

  while(1){
  float x = -73.61;
  float y = -42.5;
  float z = 0;
  float x_value = 0;
  float vv=0;
  deploy_z_probe();

  destination[X_AXIS]= x;
  destination[Y_AXIS]= y;
  destination[Z_AXIS]= z;
  prepare_move();
  st_synchronize();

  delay(1000);
  gcode_X22();

  x_value = cartesian[Z_AXIS];
  SerialUSB.println("");	
  SerialUSB.print("x_value : ");	
  SerialUSB.println(x_value);	

	//max_pos[Z_AXIS]= max_pos[Z_AXIS] - h_value;
    //set_delta_constants();
	//SerialUSB.println("");
    //SerialUSB.print(" H : ");
	//SerialUSB.println(max_pos[Z_AXIS], 4);

  
  delay(500);


   
  gcode_G28();
  delay(100);

  x = 73.61;
  y = -42.5;
  z = 0;
  float y_value=0;
  deploy_z_probe();

  destination[X_AXIS]= x;
  destination[Y_AXIS]= y;
  destination[Z_AXIS]= z;
  prepare_move();
  st_synchronize();

  delay(1000);
  gcode_X22();

  y_value = cartesian[Z_AXIS];

  //SerialUSB.println("");	
  SerialUSB.print("y_value : ");	
  SerialUSB.println(y_value);	 
  
  v_value = abs(x_value - y_value);
  SerialUSB.print("v_value : ");	
  SerialUSB.println(v_value);	

  if(x_value > y_value)
  {
    if(v_value < k_value)
    {
      endstop_adj[0] = x_value;
	  endstop_adj[1] = y_value;

	  set_delta_constants();
	  SerialUSB.println("");
      SerialUSB.print(" X : ");
	  SerialUSB.println(endstop_adj[0], 4);
	  SerialUSB.print(" Y : ");
	  SerialUSB.println(endstop_adj[1], 4);
	  break;
	
    }
    else
    {
      vv = ((y_value - x_value) * 0.8);
	           endstop_adj[1] = endstop_adj[1] - vv;
	           //endstop_adj[1] =  vv;
               set_delta_constants();
	           SerialUSB.print(" vv : ");
			   SerialUSB.println(vv);
			   SerialUSB.print("end_y : ");
               SerialUSB.println(endstop_adj[1], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
    }
  }
  
  else
  {
    if(v_value < k_value)
    {
      endstop_adj[0] = x_value;
	  endstop_adj[1] = y_value;

	  set_delta_constants();
	  SerialUSB.println("");
      SerialUSB.print(" X : ");
	  SerialUSB.println(endstop_adj[0], 4);
	  SerialUSB.print(" Y : ");
	  SerialUSB.println(endstop_adj[1], 4);
	  break;
	
    }
    else
    {
      vv = ((x_value - y_value) * 0.8);
	           endstop_adj[0] = endstop_adj[0] - vv;
	           //endstop_adj[0] =  vv;
               set_delta_constants();
	           SerialUSB.print(" vv : ");
			   SerialUSB.println(vv);
			   SerialUSB.print("end_x : ");
               SerialUSB.println(endstop_adj[0], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
    }
  }
}
}

inline void gcode_X25()
{
  float vv=0;
  //float v1_offset=0;
  //float v2_offset=0;
  float v_value=0;

  //gcode_X23();
  //delay(500);

  //gcode_X24();
  //delay(500);
  
  gcode_G28();
  delay(100);

  while(1){
  float x = 73.61;
  float y = -42.5;
  float z = 0;
  float y_value = 0;
  float vv=0;
  deploy_z_probe();

  destination[X_AXIS]= x;
  destination[Y_AXIS]= y;
  destination[Z_AXIS]= z;
  prepare_move();
  st_synchronize();

  delay(1000);
  gcode_X22();

  y_value = cartesian[Z_AXIS];
  SerialUSB.println("");	
  SerialUSB.print("y_value : ");	
  SerialUSB.println(y_value);	

	//max_pos[Z_AXIS]= max_pos[Z_AXIS] - h_value;
    //set_delta_constants();
	//SerialUSB.println("");
    //SerialUSB.print(" H : ");
	//SerialUSB.println(max_pos[Z_AXIS], 4);

  
  delay(500);


   
  gcode_G28();
  delay(100);

  x = 0;
  y = 85;
  z = 0;
  float z_value=0;
  deploy_z_probe();

  destination[X_AXIS]= x;
  destination[Y_AXIS]= y;
  destination[Z_AXIS]= z;
  prepare_move();
  st_synchronize();

  delay(1000);
  gcode_X22();

  z_value = cartesian[Z_AXIS];

  //SerialUSB.println("");	
  SerialUSB.print("z_value : ");	
  SerialUSB.println(z_value);	 
  
  v_value = abs(z_value - y_value);
  SerialUSB.print("v_value : ");	
  SerialUSB.println(v_value);	

  if(y_value > z_value)
  {
    if(v_value < k_value)
    {
      endstop_adj[1] = y_value;
	  endstop_adj[2] = z_value;

	  set_delta_constants();
	  SerialUSB.println("");
      SerialUSB.print(" Y : ");
	  SerialUSB.println(endstop_adj[1], 4);
	  SerialUSB.print(" Z : ");
	  SerialUSB.println(endstop_adj[2], 4);
	  break;
	
    }
    else
    {
      vv = ((z_value - y_value) * 0.8);
	           endstop_adj[2] = endstop_adj[2] - vv;
	           //endstop_adj[2] =   vv;
               set_delta_constants();
	           SerialUSB.print(" vv : ");
			   SerialUSB.println(vv);
			   SerialUSB.print("end_z : ");
               SerialUSB.println(endstop_adj[2], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
    }
  }
  
  else
  {
    if(v_value < k_value)
    {
      endstop_adj[1] = y_value;
	  endstop_adj[2] = z_value;

	  set_delta_constants();
	  SerialUSB.println("");
      SerialUSB.print(" Y : ");
	  SerialUSB.println(endstop_adj[1], 4);
	  SerialUSB.print(" Z : ");
	  SerialUSB.println(endstop_adj[2], 4);
	  break;
	
    }
    else
    {
      vv = ((y_value - z_value) * 0.8);
	           endstop_adj[1] = endstop_adj[1] - vv;
	           //endstop_adj[1] =  vv;
               set_delta_constants();
	           SerialUSB.print(" vv : ");
			   SerialUSB.println(vv);
			   SerialUSB.print("end_y : ");
               SerialUSB.println(endstop_adj[1], 4);
	           delay(100);
	           gcode_G28();
	           delay(100);
    }
  }
}
}

inline void gcode_X26()
{
	  float x = 0;
	  float y = 0;
	  float z = 0;
	
	  gcode_G28();
	  delay(100);
	  
	  if (code_seen('P'))
	  {
		x = -73.61;
		y = -42.5;
		z = 0;
	  }
	  
	  if (code_seen('Q'))
	  {
		x = 73.61;
		y = -42.5;
		z = 0;
	  }
	
	  if (code_seen('S'))
	  {
		x = 0;
		y = 85;
		z = 0;
	  }
	
	
	  
	  
	  float x_value = 0;
	  float vv=0;
	  deploy_z_probe();
	
	  destination[X_AXIS]= x;
	  destination[Y_AXIS]= y;
	  destination[Z_AXIS]= z;
	  prepare_move();
	  st_synchronize();
	
	  SERIAL_PROTOCOLPGM("X:");
	  SERIAL_PROTOCOL(current_position[X_AXIS]);
	  SERIAL_PROTOCOLPGM(" Y:");
	  SERIAL_PROTOCOL(current_position[Y_AXIS]);
	  SERIAL_PROTOCOLPGM(" Z:");
	  SERIAL_PROTOCOL(current_position[Z_AXIS]);
	  //SERIAL_PROTOCOLPGM(" E:");
	  //SERIAL_PROTOCOL(current_position[E_AXIS]);
	
	  SERIAL_PROTOCOLPGM(MSG_COUNT_X);
	  SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
	  SERIAL_PROTOCOLPGM(" Y:");
	  SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
	  SERIAL_PROTOCOLPGM(" Z:");
	  SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);
	
	  SERIAL_EOL;
	
	  delay(3000);
	
	  gcode_X22();
	
	
	  actuator_to_cartesian(cdelta);
	
	  SerialUSB.print("X:");
	  SerialUSB.print(cartesian[X_AXIS]);
	  SerialUSB.print(" Y:");
	  SerialUSB.print(cartesian[Y_AXIS]);
	  SerialUSB.print(" Z:");
	  SerialUSB.print(cartesian[Z_AXIS]);
	
	  SerialUSB.print("Count X:");
	  SerialUSB.print(cdelta[X_AXIS]);
	  SerialUSB.print(" Y:");
	  SerialUSB.print(cdelta[Y_AXIS]);
	  SerialUSB.print(" Z:");
	  SerialUSB.println(cdelta[Z_AXIS]);
	
	  delay(500);
	  gcode_G28();
	  delay(500);
	
	  
}

inline void gcode_X27()
{
  gcode_X23();
  delay(500);

  gcode_X24();
  delay(500);
  
  gcode_X25();
  delay(500);

  gcode_G28();
  delay(500);
}

inline void gcode_X28()
{
	  int i=0;
	
	  if (code_seen('A'))
	  {
		gcode_G28();
		delay(500);
		gcode_X23();
		delay(500);
		gcode_G28();
		delay(500);
		gcode_G29();
		delay(500);
		gcode_G28();
		delay(500);
		gcode_G30();
		delay(500);
		gcode_G28();
		delay(500);
	  }
	
	  if (code_seen('B'))
	  {
		gcode_G28();
		delay(500);
		gcode_X11();
		delay(500);
		gcode_X12(k_value);
		delay(500);
		gcode_X13(k_value);
		delay(500);
		gcode_X14(k_value);
		delay(500);
	  }
	  
	
	  for(i=0; i<10;i++)
	  {
	  float x = 0;
	  float y = 0;
	  float z = 0;
	  float x_value = 0;
	  float vv=0;
	  deploy_z_probe();
	
	  destination[X_AXIS]= x;
	  destination[Y_AXIS]= y;
	  destination[Z_AXIS]= z;
	  prepare_move();
	  st_synchronize();
	
	  delay(500);
	
	  destination[X_AXIS]= 30;
	  destination[Y_AXIS]= -30;
	  destination[Z_AXIS]= 0;
	  prepare_move();
	  st_synchronize();
	  
	  
	  for(y=-30; y<31 ;y++)
	  {
		destination[X_AXIS]= x;
		destination[Y_AXIS]= y;
		destination[Z_AXIS]= z;
		prepare_move();
		st_synchronize();
	  }
	
	  delay(500);
	  
	  for(x=30; x>-31 ;x--)
	  {
		destination[X_AXIS]= x;
		destination[Y_AXIS]= y;
		destination[Z_AXIS]= z;
		prepare_move();
		st_synchronize();  
	  }
	
	  delay(500);
	
	  for(y=30; y>-31 ;y--)
	  {
		destination[X_AXIS]= x;
		destination[Y_AXIS]= y;
		destination[Z_AXIS]= z;
		prepare_move();
		st_synchronize();  
	  }
	
	  delay(500);
	
	  for(x=-30; x<31 ;x++)
	  {
		destination[X_AXIS]= x;
		destination[Y_AXIS]= y;
		destination[Z_AXIS]= z;
		prepare_move();
		st_synchronize();  
	  }
	
	  delay(100);
	  gcode_G28();
	  delay(500);
		}
	  
}

inline void gcode_X29()
{
  int i=0;

  for(i=0; i<5;i++)
  {
  float x = 0;
  float y = 0;
  float z = 0;
  float x_value = 0;
  float vv=0;
  deploy_z_probe();

  destination[X_AXIS]= x;
  destination[Y_AXIS]= y;
  destination[Z_AXIS]= z;
  prepare_move();
  st_synchronize();

  delay(500);

  destination[X_AXIS]= 50;
  destination[Y_AXIS]= -50;
  destination[Z_AXIS]= 0;
  prepare_move();
  st_synchronize();
  
  delay(500);
  
  for(y=-50; y<51 ;y++)
  {
    destination[X_AXIS]= x;
    destination[Y_AXIS]= y;
    destination[Z_AXIS]= z;
    prepare_move();
    st_synchronize();  
  }

  delay(500);
  
  for(x=50; x>-51 ;x--)
  {
    destination[X_AXIS]= x;
    destination[Y_AXIS]= y;
    destination[Z_AXIS]= z;
    prepare_move();
    st_synchronize();  
  }

  delay(500);

  for(y=50; y>-51 ;y--)
  {
    destination[X_AXIS]= x;
    destination[Y_AXIS]= y;
    destination[Z_AXIS]= z;
    prepare_move();
    st_synchronize();  
  }

  delay(500);

  for(x=-50; x<51 ;x++)
  {
    destination[X_AXIS]= x;
    destination[Y_AXIS]= y;
    destination[Z_AXIS]= z;
    prepare_move();
    st_synchronize();  
  }

  delay(500);
  gcode_G28();
  delay(500);
  	}
  
}

inline void gcode_X30()
{
  gcode_G28();
  delay(500);
  gcode_G29();
  delay(500);


//G30 A
  int iterations;

  if (code_seen('C')) 
  {
      //Show carriage positions 
      SERIAL_ECHOLN("Carriage Positions for last scan:");
      for(int8_t i=0; i < 7; i++) {
        SERIAL_ECHO("[");
        SERIAL_ECHO(saved_positions[i][X_AXIS]);
        SERIAL_ECHO(", ");
        SERIAL_ECHO(saved_positions[i][Y_AXIS]);
        SERIAL_ECHO(", ");
        SERIAL_ECHO(saved_positions[i][Z_AXIS]);
        SERIAL_ECHOLN("]");
      }
      return;
  }
  


    //Zero the bed level array
  for (int y = 0; y < 7; y++) 
  {
    for (int x = 0; x < 7; x++) 
	{
      bed_level[x][y] = 0.0;
    }
  }

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  feedmultiply = 100;

  SerialUSB.println("Starting Auto Calibration..");      
  if (code_value() != 0) ac_prec = code_value();
  SERIAL_ECHO("Calibration precision: +/-");
  SERIAL_PROTOCOL_F(ac_prec,3);
  SERIAL_ECHOLN("mm");

      //Zero the bedlevel array in case this affects bed probing
      for (int y = 0; y >=6; y++) {
        for (int x = 0; x >=6; y++) {
          bed_level[x][y] = 0.0;
        }
      }

  home_delta_axis();
  deploy_z_probe(); 

  //Probe all points
  bed_probe_all();

  //Show calibration report      
  calibration_report();

  if (code_seen('E')) 
  {
		int iteration = 0;
		do {
		  iteration ++;
		  SerialUSB.print("Iteration: ");
		  SerialUSB.println(iteration);
		  //ECHO_LMV(DB, "Iteration: ", iteration);

		  SerialUSB.println("Checking/Adjusting endstop offsets");  
		  //ECHO_LM(DB, "Checking/Adjusting endstop offsets");
		  adj_endstops();
  
		  bed_probe_all();
		  calibration_report();
		} while ((bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
			  or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
			  or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec));
        SerialUSB.println("Endstop adjustment complete");
		//ECHO_LM(DB, "Endstop adjustment complete");
  }

  
  if (code_seen('R')) 
  {  
		int iteration = 0;
		do {
		  iteration ++;
		  SerialUSB.print("Iteration: ");
		  SerialUSB.println(iteration);
		  //ECHO_LMV(DB, "Iteration: ", iteration);

		  SerialUSB.println("Checking/Adjusting endstop offsets"); 
		  //ECHO_LM(DB, "Checking/Adjusting endstop offsets");
		  adj_endstops();
  
		  bed_probe_all();
		  calibration_report();

          SerialUSB.println("Checking delta radius"); 
		  //ECHO_LM(DB, "Checking delta radius");
		  adj_deltaradius();
  
		} while ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
			  or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
			  or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
			  or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec));
   }
	   
   if (code_seen('I')) 
   {
        SerialUSB.print("Adjusting Tower Delta for tower");
		SerialUSB.println(code_value());
		//ECHO_LMV(DB, "Adjusting Tower Delta for tower", code_value());
		adj_tower_delta(code_value());

		SerialUSB.print("Tower Delta adjustment complete");
		//ECHO_LM(DB, "Tower Delta adjustment complete");
   }
  
   if (code_seen('D')) {
   	    SerialUSB.print("Adjusting Diagonal Rod Length");
		//ECHO_LM(DB, "Adjusting Diagonal Rod Length");
		adj_diagrod_length();
		SerialUSB.print("Diagonal Rod Length adjustment complete");
		//ECHO_LM(DB, "Diagonal Rod Length adjustment complete");
	  }

  if (code_seen('A')) 
  {
      int iteration = 0;
      int dr_adjusted;

      do {
        do {
          iteration ++;
		  SerialUSB.print("Iteration: ");
		  SerialUSB.println(iteration);
          //ECHO_LMV(DB, "Iteration: ", iteration);
          SerialUSB.println("Checking/Adjusting endstop offsets");
         // ECHO_LM(DB, "Checking/Adjusting endstop offsets");
          adj_endstops();

          bed_probe_all();
          calibration_report();

          if ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)) {
		  	SerialUSB.println("Checking delta radius");
            //ECHO_LM(DB, "Checking delta radius");
            dr_adjusted = adj_deltaradius();
          }
          else dr_adjusted = 0;
          /*
          ECHO_EMV("bed_level_c=", bed_level_c, 4);
          ECHO_EMV("bed_level_x=", bed_level_x, 4);
          ECHO_EMV("bed_level_y=", bed_level_y, 4);
          ECHO_EMV("bed_level_z=", bed_level_z, 4);
          */
        } while ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
              or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
              or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
              or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)
              or (dr_adjusted != 0));

        if ((bed_level_ox < -ac_prec) or (bed_level_ox > ac_prec) or
            (bed_level_oy < -ac_prec) or (bed_level_oy > ac_prec) or
            (bed_level_oz < -ac_prec) or (bed_level_oz > ac_prec)) {
          SerialUSB.println("Checking for tower geometry errors..");  
          //ECHO_LM(DB, "Checking for tower geometry errors..");
          if (fix_tower_errors() != 0 ) {
            // Tower positions have been changed .. home to endstops
            SerialUSB.println("Tower Positions changed .. Homing Endstops");
            //ECHO_LM(DB, "Tower Positions changed .. Homing Endstops");
            home_delta_axis();
            bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
          }
          else {
		  	SerialUSB.println("Checking DiagRod Length");
            //ECHO_LM(DB, "Checking DiagRod Length");
            if (adj_diagrod_length() != 0) { 
              //If diag rod length has been changed .. home to endstops
              SerialUSB.println("Diagonal Rod Length changed .. Homing Endstops");
              //ECHO_LM(DB, "Diagonal Rod Length changed .. Homing Endstops");
              home_delta_axis();
              bed_safe_z = Z_RAISE_BETWEEN_PROBINGS - z_probe_offset[Z_AXIS];
            }
          }
          bed_probe_all();
          calibration_report();
        }
        /*
        ECHO_EMV("bed_level_c=", bed_level_c, 4);
        ECHO_EMV("bed_level_x=", bed_level_x, 4);
        ECHO_EMV("bed_level_y=", bed_level_y, 4);
        ECHO_EMV("bed_level_z=", bed_level_z, 4);
        ECHO_EMV("bed_level_ox=", bed_level_ox, 4);
        ECHO_EMV("bed_level_oy=", bed_level_oy, 4);
        ECHO_EMV("bed_level_oz=", bed_level_oz, 4);
        */
      } while((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
           or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
           or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
           or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)
           or (bed_level_ox < -ac_prec) or (bed_level_ox > ac_prec)
           or (bed_level_oy < -ac_prec) or (bed_level_oy > ac_prec)
           or (bed_level_oz < -ac_prec) or (bed_level_oz > ac_prec));
      SerialUSB.println("Autocalibration Complete");
      //ECHO_LM(DB, "Autocalibration Complete");
    }



  if (code_seen('O')) 
  {
  iterations = 100; //Maximum number of iterations
  int loopcount = 1;
  float adj_r_target, adj_dr_target;
  float adj_r_target_delta = 0, adj_dr_target_delta = 0;
  float adj_AlphaA, adj_AlphaB, adj_AlphaC;
  float adj_RadiusA, adj_RadiusB, adj_RadiusC;
  float radiusErrorA, radiusErrorB,radiusErrorC;
  float adj_r = 0, adj_dr = 0;
  boolean equalAB, equalBC, equalCA;
  boolean adj_r_done, adj_dr_done, adj_tower_done;
  boolean adj_dr_allowed = true;
  float h_endstop = -100, l_endstop = 100;
  float probe_error, ftemp; 

  //Check that endstop are within limits
      if (bed_level_x + endstop_adj[0] > h_endstop) h_endstop = bed_level_x + endstop_adj[0];
      if (bed_level_x + endstop_adj[0] < l_endstop) l_endstop = bed_level_x + endstop_adj[0];
      if (bed_level_y + endstop_adj[1] > h_endstop) h_endstop = bed_level_y + endstop_adj[1];
      if (bed_level_y + endstop_adj[1] < l_endstop) l_endstop = bed_level_y + endstop_adj[1];
      if (bed_level_z + endstop_adj[2] > h_endstop) h_endstop = bed_level_z + endstop_adj[2];
      if (bed_level_z + endstop_adj[2] < l_endstop) l_endstop = bed_level_z + endstop_adj[2];

      if (h_endstop - l_endstop > 3) {
        SERIAL_ECHOLN("The position of the endstop switches on this printer are not within limits");
        SERIAL_ECHOLN("Adjust endstop switches so that they are within 3mm Z-height of each other");
        SERIAL_EOL;
        SERIAL_ECHOPAIR("Current Endstop Positions - X: ", bed_level_x + endstop_adj[0]); 
        SERIAL_ECHOPAIR(" Y: ", bed_level_y + endstop_adj[1]);
        SERIAL_ECHOPAIR(" Z: ", bed_level_z + endstop_adj[2]);
        SERIAL_EOL;
        SERIAL_EOL;
        SERIAL_ECHOLN("Auto calibration aborted");

        retract_z_probe();

        //Restore saved variables
        feedrate = saved_feedrate;
        feedmultiply = saved_feedmultiply;
        return;
      }

     
	 do {
        SERIAL_ECHO("Iteration: ");
        SERIAL_ECHO(loopcount);
        SERIAL_EOL;

        if ((bed_level_c > 3) or (bed_level_c < -3)) {
          //Build height is not set correctly .. 
          max_pos[Z_AXIS] -= bed_level_c + 2;
          set_delta_constants();
          SERIAL_ECHOPAIR("Adjusting Z-Height to: ", max_pos[Z_AXIS]);
          SERIAL_ECHOLN(" mm..");
        } 
        else {
          if ((bed_level_x < -ac_prec) or (bed_level_x > ac_prec) or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec) or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)) {
            //Endstop req adjustment
            SERIAL_ECHOLN("Adjusting Endstop..");
            endstop_adj[0] += bed_level_x / 1.05;
            endstop_adj[1] += bed_level_y / 1.05;
            endstop_adj[2] += bed_level_z / 1.05; 

            //Check that no endstop adj values are > 0 (not allowed).. if they are, reduce the build height to compensate.
            h_endstop = 0;
            for(int x=0; x < 3; x++) { 
              if (endstop_adj[x] > h_endstop) h_endstop = endstop_adj[x]; 
            }
            if (h_endstop > 0) {
              //Reduce build height and adjust endstop
              for(int x=0; x < 3; x++) {
                endstop_adj[x] -= h_endstop + 2;
              }
              max_pos[Z_AXIS] -= h_endstop + 2;
              set_delta_constants();
              SERIAL_ECHOPAIR("Adjusting Z-Height to: ", max_pos[Z_AXIS]);
              SERIAL_ECHOLN(" mm..");
			  
            }
          }
          else {
            SERIAL_ECHOLN("Endstop: OK");
            adj_r_target = (bed_level_x + bed_level_y + bed_level_z) / 3;
            adj_dr_target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;

            //Determine which parameters require adjustment
            if ((bed_level_c >= adj_r_target - ac_prec) and (bed_level_c <= adj_r_target + ac_prec)) adj_r_done = true; 
            else adj_r_done = false;
            if ((adj_dr_target >= adj_r_target - ac_prec) and (adj_dr_target <= adj_r_target + ac_prec)) adj_dr_done = true; 
            else adj_dr_done = false;
            if ((bed_level_x != bed_level_ox) or (bed_level_y != bed_level_oy) or (bed_level_z != bed_level_oz)) adj_tower_done = false; 
            else adj_tower_done = true;
            if ((adj_r_done == false) or (adj_dr_done == false) or (adj_tower_done == false)) {
              //delta geometry adjustment required
              SERIAL_ECHOLN("Adjusting Delta Geometry..");
              //set initial direction and magnitude for delta radius & diagonal rod adjustment
              if (adj_r == 0) {
                if (adj_r_target > bed_level_c) adj_r = 1; 
                else adj_r = -1;
              }

              if (adj_dr == 0) {
                if (adj_r_target > adj_dr_target) adj_dr = 1; 
                else adj_dr = -1;
              }

              //Don't adjust tower positions on first iteration
              adj_AlphaA = adj_AlphaB = adj_AlphaC = 0; 
              adj_RadiusA = adj_RadiusB = adj_RadiusC = 0;

              do {   
                //Apply adjustments 
                if (adj_r_done == false) {
                  SERIAL_ECHOPAIR("Adjusting Delta Radius (",delta_radius);
                  SERIAL_ECHOPAIR(" -> ", delta_radius + adj_r);
                  SERIAL_ECHOLN(")");
                  delta_radius += adj_r;
                }

                if (adj_dr_allowed == false) adj_dr_done = true;
                if (adj_dr_done == false) {
                  SERIAL_ECHOPAIR("Adjusting Diagonal Rod Length (",delta_diagonal_rod);
                  SERIAL_ECHOPAIR(" -> ", delta_diagonal_rod + adj_dr);
                  SERIAL_ECHOLN(")");
                  delta_diagonal_rod += adj_dr;
                }

                tower_adj[0] -= adj_AlphaA;
                tower_adj[1] -= adj_AlphaB;
                tower_adj[2] -= adj_AlphaC;
                tower_adj[3] += adj_RadiusA;
                tower_adj[4] += adj_RadiusB;
                tower_adj[5] += adj_RadiusC;

                set_delta_constants();

                bed_probe_all();
                calibration_report();

                //Check to see if auto calc is complete to within limits..
                if (adj_dr_allowed == true) {
                  if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
                    and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
                    and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
                    and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)
                    and (bed_level_ox >= -ac_prec) and (bed_level_ox <= ac_prec)
                    and (bed_level_oy >= -ac_prec) and (bed_level_oy <= ac_prec)
                    and (bed_level_oz >= -ac_prec) and (bed_level_oz <= ac_prec)) loopcount = iterations;
                } 
                else {
                  if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
                    and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
                    and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
                    and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) loopcount = iterations;
                }

                //set delta radius and diagonal rod targets
                adj_r_target = (bed_level_x + bed_level_y + bed_level_z) / 3;
                adj_dr_target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;

                //set Tower position adjustment values
                adj_AlphaA = bed_level_oy - bed_level_oz;
                adj_AlphaB = bed_level_oz - bed_level_ox;
                adj_AlphaC = bed_level_ox - bed_level_oy;

                //set tower radius errors
                radiusErrorA = bed_level_x - bed_level_ox;
                radiusErrorB = bed_level_y - bed_level_oy;
                radiusErrorC = bed_level_z - bed_level_oz;

                if ((radiusErrorA >= (radiusErrorB - 0.02)) and (radiusErrorA <= (radiusErrorB + 0.02))) equalAB = true;
                else equalAB = false;
                if ((radiusErrorB >= (radiusErrorC - 0.02)) and (radiusErrorB <= (radiusErrorC + 0.02))) equalBC = true;
                else equalBC = false;
                if ((radiusErrorC >= (radiusErrorA - 0.02)) and (radiusErrorC <= (radiusErrorA + 0.02))) equalCA = true;
                else equalCA = false;

                #ifdef DEBUG_MESSAGES
                  if (equalAB == true) {
                    SERIAL_ECHOPAIR("Tower AB Equal (A=",radiusErrorA);
                    SERIAL_ECHOPAIR(" B=",radiusErrorB);
                    SERIAL_ECHOLN(")");
                  }
                  else SERIAL_ECHOLN("equalAB=false");

                  if (equalBC == true) {
                    SERIAL_ECHOPAIR("Tower BC Equal (B=",radiusErrorB);
                    SERIAL_ECHOPAIR(" C=",radiusErrorC);
                    SERIAL_ECHOLN(")");
                  }
                  else SERIAL_ECHOLN("equalBC=false");

                  if (equalCA == true) {
                    SERIAL_ECHOPAIR("Tower CA Equal (C=",radiusErrorC);
                    SERIAL_ECHOPAIR(" A=",radiusErrorA);
                    SERIAL_ECHOLN(")");
                  }
                  else SERIAL_ECHOLN("equalCA=false");
                #endif //DEBUG_MESSAGES

                if ((equalAB == true) and (equalBC == true) and (equalCA == true)) {
                  // all tower radius out by the same amount (within 0.02) - allow adjustment with delta rod length
                  #ifdef DEBUG_MESSAGES
                    SERIAL_ECHOLN("All tower radius errors equal");
                  #endif
                  adj_RadiusA = adj_RadiusB = adj_RadiusC = 0;
                }

                if ((equalAB == true) and (equalBC == false) and (equalCA == false)) {
                  //Tower C radius error.. adjust it
                  SERIAL_ECHOLN("TowerC Radius error - adjusting");
                  if (adj_RadiusC == 0) {
                    if (bed_level_z < bed_level_oz) adj_RadiusC = 0.5;
                    if (bed_level_z > bed_level_oz) adj_RadiusC = -0.5;
                    #ifdef DEBUG_MESSAGES
                      SERIAL_ECHOPAIR("adj_RadiusC set to ",adj_RadiusC);
                      SERIAL_EOL;
                    #endif
                  }
                }

                if ((equalBC == true) and (equalAB == false) and (equalCA == false)) {
                  //Tower A radius error .. adjust it
                  SERIAL_ECHOLN("TowerA Radius error - adjusting");
                  if (adj_RadiusA == 0) {
                    if (bed_level_x < bed_level_ox) adj_RadiusA = 0.5;
                    if (bed_level_x > bed_level_ox) adj_RadiusA = -0.5;  
                    #ifdef DEBUG_MESSAGES
                      SERIAL_ECHOPAIR("adj_RadiusA set to ",adj_RadiusA);
                      SERIAL_EOL;
                    #endif
                  }
                } 

                if ((equalCA == true) and (equalAB == false) and (equalBC == false)) {
                  //Tower B radius error .. adjust it
                  SERIAL_ECHOLN("TowerB Radius error - adjusting");
                  if (adj_RadiusB == 0) {
                    if (bed_level_y < bed_level_oy) adj_RadiusB = 0.5;
                    if (bed_level_y > bed_level_oy) adj_RadiusB = -0.5;                     
                    #ifdef DEBUG_MESSAGES
                      SERIAL_ECHOPAIR("adj_RadiusB set to ",adj_RadiusB);
                      SERIAL_EOL;
                    #endif
                  }
                }                   

                if (((adj_r > 0) and (bed_level_c > adj_r_target)) or ((adj_r < 0) and (bed_level_c < adj_r_target))) {
                  //overshot target .. reverse & scale down
                  adj_r = -(adj_r / 2);
                }

                if (((adj_dr > 0) and (adj_dr_target > adj_r_target)) or ((adj_dr < 0) and (adj_dr_target < adj_r_target))) {
                  //overshot target .. reverse & scale down
                  adj_dr = -(adj_dr / 2);
                }

                //Tower radius overshot targets?
                if (((adj_RadiusA > 0) and (bed_level_x > bed_level_ox)) or ((adj_RadiusA < 0) and (bed_level_x < bed_level_ox))) adj_RadiusA = -(adj_RadiusA / 2);
                if (((adj_RadiusB > 0) and (bed_level_y > bed_level_oy)) or ((adj_RadiusB < 0) and (bed_level_y < bed_level_oy))) adj_RadiusB = -(adj_RadiusB / 2);
                if (((adj_RadiusC > 0) and (bed_level_z > bed_level_oz)) or ((adj_RadiusC < 0) and (bed_level_z < bed_level_oz))) adj_RadiusC = -(adj_RadiusC / 2);

                //Delta radius adjustment complete?                       
                if ((bed_level_c >= (adj_r_target - ac_prec)) and (bed_level_c <= (adj_r_target + ac_prec))) adj_r_done = true; 
                else adj_r_done = false;

                //Diag Rod adjustment complete?
                if ((adj_dr_target >= (adj_r_target - ac_prec)) and (adj_dr_target <= (adj_r_target + ac_prec))) adj_dr_done = true; 
                else adj_dr_done = false;

                #ifdef DEBUG_MESSAGES
                  SERIAL_ECHOPAIR("c: ", bed_level_c);
                  SERIAL_ECHOPAIR(" x: ", bed_level_x);
                  SERIAL_ECHOPAIR(" y: ", bed_level_y);
                  SERIAL_ECHOPAIR(" z: ", bed_level_z);
                  SERIAL_ECHOPAIR(" ox: ", bed_level_ox);
                  SERIAL_ECHOPAIR(" oy: ", bed_level_oy);
                  SERIAL_ECHOPAIR(" oz: ", bed_level_oz);
                  SERIAL_EOL;
                  SERIAL_ECHO("radius:");
                  SERIAL_PROTOCOL_F(delta_radius, 4);
                  SERIAL_ECHO(" diagrod:");
                  SERIAL_PROTOCOL_F(delta_diagonal_rod, 4);
                  SERIAL_EOL;
                  SERIAL_ECHO("Radius Adj Complete: ");
                  if (adj_r_done == true) SERIAL_ECHO("Yes"); 
                  else SERIAL_ECHO("No");
                  SERIAL_ECHO(" DiagRod Adj Complete: ");
                  if (adj_dr_done == true) SERIAL_ECHO("Yes"); 
                  else SERIAL_ECHO("No");
                  SERIAL_EOL;
                  SERIAL_ECHOPAIR("RadiusA Error: ",radiusErrorA);
                  SERIAL_ECHOPAIR(" (adjust: ",adj_RadiusA);
                  SERIAL_ECHOLN(")");
                  SERIAL_ECHOPAIR("RadiusB Error: ",radiusErrorB);
                  SERIAL_ECHOPAIR(" (adjust: ",adj_RadiusB);
                  SERIAL_ECHOLN(")");
                  SERIAL_ECHOPAIR("RadiusC Error: ",radiusErrorC);
                  SERIAL_ECHOPAIR(" (adjust: ",adj_RadiusC);
                  SERIAL_ECHOLN(")");
                  SERIAL_ECHOPAIR("DeltaAlphaA: ",adj_AlphaA);
                  SERIAL_EOL;
                  SERIAL_ECHOPAIR("DeltaAlphaB: ",adj_AlphaB);
                  SERIAL_EOL;
                  SERIAL_ECHOPAIR("DeltaAlphaC: ",adj_AlphaC);
                  SERIAL_EOL;
                #endif
              } while (((adj_r_done == false) or (adj_dr_done = false)) and (loopcount < iterations));
            }
            else {
              SERIAL_ECHOLN("Delta Geometry: OK");
            }
          }
        }

        if (loopcount < iterations) {
          delay(500);
          home_delta_axis();

          //probe bed and display report
          bed_probe_all();

		  
          calibration_report();
          
          //Check to see if autocalc is complete to within limits..
          if (adj_dr_allowed == true) {
            if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
              and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
              and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
              and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)
              and (bed_level_ox >= -ac_prec) and (bed_level_ox <= ac_prec)
              and (bed_level_oy >= -ac_prec) and (bed_level_oy <= ac_prec)
              and (bed_level_oz >= -ac_prec) and (bed_level_oz <= ac_prec)) loopcount = iterations;
          }
          else {
            if   ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)
              and (bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)
              and (bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)
              and (bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) loopcount = iterations;
          }
        }
        loopcount ++;
      }while(loopcount < iterations);

      SERIAL_ECHOLN("Auto Calibration Complete");
      LCD_MESSAGEPGM("Complete");
      SERIAL_ECHOLN("Issue M500 Command to save calibration settings to EPROM (if enabled)");
      /*   
       if ((abs(delta_diagonal_rod - saved_delta_diagonal_rod) > 1) and (adj_dr_allowed == true)) {
       SERIAL_EOL;
       SERIAL_ECHOPAIR("WARNING: The length of diagonal rods specified (", saved_delta_diagonal_rod);
       SERIAL_ECHOLN(" mm) appears to be incorrect");
       SERIAL_ECHOLN("If you have measured your rods and you believe that this value is correct, this could indicate");
       SERIAL_ECHOLN("excessive twisting movement of carriages and/or loose screws/joints on carriages or end effector");
       }
       */
    

    retract_z_probe();

    //Restore saved variables
    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
   
//G30 A END

    delay(500);
    gcode_G28();
	delay(500);
	gcode_X29();
	delay(500);
  }
}	

#endif

inline void gcode_X8()
{
  if (code_seen('O')) 
  {
    filrunout_flag = 1;
	filrunout_info_count=0;
  }

  if (code_seen('F')) 
  {
    filrunout_flag = 0;
	filrunout_info_count=0;
  }
}



inline void gcode_X111()
{
  SerialUSB.print("FW Version:");
  SerialUSB.println(FW_V,4);
}



/*****************************************************
*** Process Commands and dispatch them to handlers ***
******************************************************/
void process_commands() 
{
  if(code_seen('G'))
  {
    int gCode = code_value_short();
    switch(gCode) 
    {
      //G0 -> G1
      case 0:
      case 1:
        gcode_G0_G1();
        break;

      // G2, G3
      #ifndef SCARA
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(gCode == 2);
          break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4(); 
        break;

      #ifdef FWRETRACT
        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(gCode == 10);
          break;
      #endif //FWRETRACT

      case 28: //G28: Home all axes, one at a time
        gcode_G28(); 
        break;

      #ifdef ENABLE_AUTO_BED_LEVELING

#if 1 //aven 0724 Mark as not used
		case 29: // G29 Detailed Z-Probe, probes the bed at 3 or more points.
          gcode_G29();
		  break;
#endif

        #ifndef Z_PROBE_SLED
          case 30: // G30 Single Z Probe
            gcode_G30();
            break;
        #else // Z_PROBE_SLED
          case 31: // G31: dock the sled
          case 32: // G32: undock the sled
            dock_sled(gCode == 31);
        #endif // Z_PROBE_SLED
      #endif // ENABLE_AUTO_BED_LEVELING

      #ifdef DELTA

#if 1 //aven 0724 Mark as not used	  
        case 29: // G29 Detailed Z-Probe, probes the bed at more points.
          gcode_G29(); break;
        case 30:  // G30 Delta AutoCalibration
          gcode_G30(); break;
#endif
		  
      #endif //DELTA

      case 60: // G60 Store in memory actual position
        gcode_G60(); 
        break;
      case 61: // G61 move to X Y Z in memory
        gcode_G61(); 
        break;
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        gcode_G92();
        break;
    }

    SERIAL_PROTOCOLLN(MSG_OK);
  }

  else if(code_seen('M')) 
  {
    int gCode = code_value_short();	
	  //SerialUSB.print("ok@M");
	  //SerialUSB.println(gCode);

	
    //switch(code_value_short()) {
    switch(gCode) 
    {
        #ifdef ULTIPANEL
        case 0: //M0 - Unconditional stop - Wait for user button press on LCD
        case 1: //M1 - Conditional stop - Wait for user button press on LCD
          gcode_M0_M1();
          break;
        #endif //ULTIPANEL

        #ifdef LASERBEAM
        case 3: // M03 S - Setting laser beam
          gcode_M3();
          break;
        case 4: // M04 - Turn on laser beam
          gcode_M4();
          break;
        case 5: // M05 - Turn off laser beam
          gcode_M5();
          break;
        #endif //LASERBEAM

      #ifdef FILAMENT_END_SWITCH
        case 11: //M11 - Start printing
          gcode_M11();
          break;
      #endif

      case 17: //M17 - Enable/Power all stepper motors
        gcode_M17();
        break;

#if 0 //aven 0724 Mark as not used
      #ifdef SDSUPPORT
        case 20: // M20 - list SD card
          gcode_M20(); 
		  break;
        case 21: // M21 - init SD card
          gcode_M21(); break;
        case 22: //M22 - release SD card
          gcode_M22(); break;
        case 23: //M23 - Select file
          gcode_M23(); break;
        case 24: //M24 - Start SD print
          gcode_M24(); break;
        case 25: //M25 - Pause SD print
          gcode_M25(); break;
        case 26: //M26 - Set SD index
          gcode_M26(); break;
        case 27: //M27 - Get SD status
          gcode_M27(); break;
        case 28: //M28 - Start SD write
          gcode_M28(); break;
        case 29: //M29 - Stop SD write
          gcode_M29(); break;
        case 30: //M30 <filename> Delete File
          gcode_M30(); break;
        case 32: //M32 - Select file and start SD print
          gcode_M32(); break;
        case 928: //M928 - Start SD write
          gcode_M928(); break;
      #endif //SDSUPPORT
#endif

      case 31: //M31 take time since the start of the SD print or an M109 command
        gcode_M31();
		break;
		
      case 42: //M42 -Change pin status via gcode
        gcode_M42();
		break;

      #if defined(ENABLE_AUTO_BED_LEVELING) && defined(Z_PROBE_REPEATABILITY_TEST)
        case 49: //M49 Z-Probe repeatability
          gcode_M49();
		  break;
		  
      #endif //defined(ENABLE_AUTO_BED_LEVELING) && defined(Z_PROBE_REPEATABILITY_TEST)

      #if HAS_POWER_SWITCH
        case 80: //M80 - Turn on Power Supply
          gcode_M80();
		  break;
		  
      #endif //HAS_POWER_SWITCH
      
      case 81: // M81 - Turn off Power, including Power Supply, if possible
        gcode_M81();
        break;
      case 82:
        gcode_M82();
        break;
      case 83:
        gcode_M83();
        break;
      case 84: // M84
        gcode_M18_M84();
        break;
      case 85: // M85
        gcode_M85();
        break;
      case 92: // M92
        gcode_M92();
        break;  
      // NOTE: NOT use
      // case 104: // M104
      //   gcode_M104();
      //   break;
      // case 105: // M105 Read current temperature
      //   gcode_M105()
      //   break;
      //
      #if HAS_FAN
        case 106: //M106 Fan On
          gcode_M106();
          break;
        case 107: //M107 Fan Off
          gcode_M107();
          break;
      #endif //FAN_PIN

      case 109: // M109 Wait for temperature
        gcode_M109(); 
        break;
      case 111: // M111 - Debug mode
        gcode_M111();
        break;
      case 112: //  M112 Emergency Stop
        gcode_M112();
        break;
      case 114: // M114
        gcode_M114();
        break;
      case 115: // M115
        gcode_M115();
        break;  
      case 117: // M117 display message
        gcode_M117();
        break;  
      case 119: // M119
        gcode_M119();
        break;  
      case 120: // M120
        gcode_M120();
        break;
      case 121: // M121
        gcode_M121();
        break;  

      #ifdef BARICUDA
        // PWM for HEATER_1_PIN
        #if defined(HEATER_1_PIN) && HEATER_1_PIN > -1
          case 126: // M126 valve open
            gcode_M126();
            break;
          case 127: // M127 valve closed
            gcode_M127();
            break;
        #endif //HEATER_1_PIN

        // PWM for HEATER_2_PIN
        #if defined(HEATER_2_PIN) && HEATER_2_PIN > -1
          case 128: // M128 valve open
            gcode_M128();
            break;
          case 129: // M129 valve closed
            gcode_M129();
            break;
        #endif //HEATER_2_PIN
      #endif //BARICUDA

      case 140: // M140 Set bed temp
        gcode_M140();
        break;

      #ifdef BLINKM
        case 150: // M150
          gcode_M150();
          break;
      #endif //BLINKM

      #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
        case 190: // M190 - Wait for bed heater to reach target.
          gcode_M190();
          break;
      #endif //TEMP_BED_PIN

      case 200: // M200 D<millimetres> set filament diameter and set E axis units to cubic millimetres (use S0 to set back to millimeters).
        gcode_M200();
        break;
      case 201: // M201
        gcode_M201();
        break;
      case 203: // M203 max feedrate mm/sec
        gcode_M203();
        break;
      case 204: // M204 acceleration S normal moves T filament only moves
        gcode_M204();
        break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
        gcode_M205();
        break;
      case 206: // M206 additional homing offset
        gcode_M206();
        break;

      #ifdef FWRETRACT
        case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
          gcode_M207();
          break;
        case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
          gcode_M208();
          break;
        case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
          gcode_M209();
          break;
      #endif // FWRETRACT

      #if HOTENDS > 1
        case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
          gcode_M218();
          break;
      #endif

      case 220: // M220 S<factor in percent>- set speed factor override percentage
        gcode_M220();
          break;
      case 221: // M221 S<factor in percent>- set extrude factor override percentage
        gcode_M221();
        break;
      case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226();
        break;

      #if defined(CHDK) || (defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1)
        case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
        gcode_M240();
        break;
      #endif // CHDK || PHOTOGRAPH_PIN

      #if defined(DOGLCD) && LCD_CONTRAST >= 0
        case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
          gcode_M250();
          break;
      #endif // DOGLCD

      #if NUM_SERVOS > 0
        case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
          gcode_M280();
          break;
      #endif // NUM_SERVOS > 0

      #if defined(LARGE_FLASH) && (BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER))
        case 300: // M300 - Play beep tone
          gcode_M300();
          break;
      #endif // LARGE_FLASH && (BEEPER>0 || ULTRALCD || LCD_USE_I2C_BUZZER)

      #ifdef PIDTEMP
        case 301: // M301
          gcode_M301();
          break;
      #endif // PIDTEMP

      #ifdef PREVENT_DANGEROUS_EXTRUDE
        case 302: // allow cold extrudes, or set the minimum extrude temperature
          gcode_M302();
          break;
      #endif // PREVENT_DANGEROUS_EXTRUDE

      case 303: // M303 PID autotune
        gcode_M303();
        break;

      #ifdef PIDTEMPBED
        case 304: // M304
          gcode_M304();
          break;
      #endif // PIDTEMPBED

      #if HAS_MICROSTEPS
        case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350();
          break;
        case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351();
          break;
      #endif // HAS_MICROSTEPS

      #ifdef SCARA
        case 360:  // M360 SCARA Theta pos1
          if (gcode_M360()) return; break;
        case 361:  // M361 SCARA Theta pos2
          if (gcode_M361()) return; break;
        case 362:  // M362 SCARA Psi pos1
          if (gcode_M362()) return; break;
        case 363:  // M363 SCARA Psi pos2
          if (gcode_M363()) return; break;
        case 364:  // M364 SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return; break;
        case 365: // M365 Set SCARA scaling for X Y Z
          gcode_M365(); break;
      #endif // SCARA

      case 400: // M400 finish all moves
        gcode_M400();
        break;

      #if defined(ENABLE_AUTO_BED_LEVELING) && (defined(SERVO_ENDSTOPS) || defined(Z_PROBE_ALLEN_KEY)) && not defined(Z_PROBE_SLED)
        case 401:
          gcode_M401();
          break;
        case 402:
          gcode_M402();
          break;
      #endif

       #ifdef FILAMENT_SENSOR
        case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404();
          break;
        case 405:  //M405 Turn on filament sensor for control
          gcode_M405();
          break;
        case 406:  //M406 Turn off filament sensor for control
          gcode_M406();
          break;
        case 407:   //M407 Display measured filament diameter
          gcode_M407();
          break;
      #endif // FILAMENT_SENSOR 

      case 500: // M500 Store settings in EEPROM
        gcode_M500();
        break;
      case 501: // M501 Read settings from EEPROM
        gcode_M501();
        break;
      case 502: // M502 Revert to default settings
        gcode_M502();
        break;
      case 503: // M503 print settings currently in memory
        gcode_M503();
        break;

      #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
        case 540:
          gcode_M540();
          break;
      #endif

       #ifdef FILAMENTCHANGEENABLE
        case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600();
          break;
      #endif // FILAMENTCHANGEENABLE

      #ifdef DUAL_X_CARRIAGE
        case 605:
          gcode_M605();
          break;
      #endif // DUAL_X_CARRIAGE

      #if defined(ENABLE_AUTO_BED_LEVELING) || defined(DELTA)
        case 666: //M666 Set Z probe offset or set delta endstop and geometry adjustment
          gcode_M666();
          break;
      #endif //defined(ENABLE_AUTO_BED_LEVELING) || defined(DELTA)

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907();
        break;

      #if HAS_DIGIPOTSS
        case 908: // M908 Control digital trimpot directly.
          gcode_M908();
          break;
      #endif // HAS_DIGIPOTSS

      #ifdef NPR2
        case 997: // M997 Cxx Move Carter xx gradi
          gcode_M997();
          break;
      #endif // NPR2

       case 999: // M999: Restart after being Stopped
         gcode_M999();
         break;

        #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
        case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
          gcode_SET_Z_PROBE_OFFSET();
          break;
      #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
    }

    SERIAL_PROTOCOLLN(MSG_OK);
  }

  else if (code_seen('T')) 
  {
     gcode_T();
     SERIAL_PROTOCOLLN(MSG_OK);
  }


//aven_0415_2015 add cmd for S_LSA1 & S_LSA2 on/off start
  else if (code_seen('X')) 
  {
    //SerialUSB.print("ok");
      int gCode = code_value_short();	
	  //SerialUSB.print("ok@X");
	  //SerialUSB.println(gCode);
    //switch(code_value_short()) 
      switch(gCode) 
      {
        case 0: 
        case 1: 
          gcode_X1();
		  break;
		case 2:   
          gcode_X2();
		  break;
        case 3:   
          gcode_X3();
		  break;
		case 4:   
          gcode_X4();
		  break;
		case 5:   
          gcode_X5();
		  break; //heater PID on/off 
        case 6:   
          gcode_X6();
		  break;
		case 7:   
          gcode_X7();
		  break;		  
		case 8:   
          gcode_X8();
		  break;		  
        case 111:   
          gcode_X111();
		  break;
 #if 0		  
		case 10:   
          gcode_X10();
		  break; 
		case 11:   
          gcode_X11();
		  break;
		case 12:   
          gcode_X12(k_value);
		  break;  
		case 13:   
          gcode_X13(k_value);
		  break;
		case 14:   
          gcode_X14(k_value);
		  break;
		case 15:   
          gcode_X15();
		  break;
		case 17:   
      gcode_X17();
		  break;
        case 18:   
          gcode_X18();
          break;
		 case 19:   
          gcode_X19();
          break; 
		  case 20:
          gcode_X20();
		  break; 
		  case 21:   
          gcode_X21();
		  break; 

		  case 22:   
          gcode_X22();
		  break; 

		  case 23:   
          gcode_X23();
		  break; 

		  case 24:   
          gcode_X24();
		  break; 

		  case 25:   
          gcode_X25();
		  break;

		  case 26:   
          gcode_X26();
		  break; 

		  case 27:   
          gcode_X27();
		  break; 

		  case 28:   
          gcode_X28();
		  break; 

		  case 29:   
          gcode_X29();
		  break; 

		  case 30:   
          gcode_X30();
		  break; 
		#endif  

		default:
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
      SERIAL_ECHO(cmdbuffer[bufindr]);
      SERIAL_ECHOLNPGM("\"");

      SERIAL_PROTOCOLLN("ER BAD_CMD");
      break;
    }

    SERIAL_PROTOCOLLN(MSG_OK);
  }
//aven_0415_2015 add cmd for S_LSA1 & S_LSA2 on/off end
  else if(code_seen('C')) {
    int cCode = code_value_short();

    switch(cCode) {
      case 1:
        // Enable/Disable lineno and sumcheck
        gcode_C1();
        break;
      default:
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
        SERIAL_ECHO(cmdbuffer[bufindr]);
        SERIAL_ECHOLNPGM("\"");

        SERIAL_PROTOCOLLN("ER BAD_CMD");
        break;
    }
  }
  else 
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(cmdbuffer[bufindr]);
    SERIAL_ECHOLNPGM("\"");

    SERIAL_PROTOCOLLN("ER BAD_CMD");
  }

  ClearToSend();
}

void FlushSerialRequestResend() {
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend() {
  refresh_cmd_timeout();
  #ifdef SDSUPPORT
    if (fromsd[bufindr]) return;
  #endif
  //SERIAL_PROTOCOLLNPGM(MSG_OK);
}

void get_coordinates() {
  for (int i = 0; i < NUM_AXIS; i++) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value() + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }
  if (code_seen('F')) {
    next_feedrate = code_value();
    if (next_feedrate > 0.0) feedrate = next_feedrate;
  }

  #ifdef LASERBEAM
    if(code_seen('L')) {
      laser_ttl_modulation = constrain(code_value(), 0, 255);
    }
  #endif // LASERBEAM
}

void get_arc_coordinates() {
  #ifdef SF_ARC_FIX
    bool relative_mode_backup = relative_mode;
    relative_mode = true;
  #endif
    get_coordinates();
  #ifdef SF_ARC_FIX
    relative_mode = relative_mode_backup;
  #endif

  offset[0] = code_seen('I') ? code_value() : 0;
  offset[1] = code_seen('J') ? code_value() : 0;
}

void clamp_to_software_endstops(float target[3]) {
  if (min_software_endstops) {
    if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
    if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
    
    float negative_z_offset = 0;
    #ifdef ENABLE_AUTO_BED_LEVELING
      if (Z_PROBE_OFFSET_FROM_EXTRUDER < 0) negative_z_offset = negative_z_offset + Z_PROBE_OFFSET_FROM_EXTRUDER;
      if (home_offset[Z_AXIS] < 0) negative_z_offset = negative_z_offset + home_offset[Z_AXIS];
    #endif
    
    if (target[Z_AXIS] < min_pos[Z_AXIS]+negative_z_offset) target[Z_AXIS] = min_pos[Z_AXIS]+negative_z_offset;
  }

  if (max_software_endstops) {
    if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
    if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
    if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
  }
}

void prepare_move() {

#if 0 //aven_0813
  #ifdef IDLE_OOZING_PREVENT || EXTRUDER_RUNOUT_PREVENT
    axis_is_moving = true;
  #endif
#endif


  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

#if 0 //aven_0813
  #ifdef SCARA //for now same as delta-code

    float difference[NUM_AXIS];
    for (int8_t i = 0; i < NUM_AXIS; i++) difference[i] = destination[i] - current_position[i];

    float cartesian_mm = sqrt(  sq(difference[X_AXIS]) +
                                sq(difference[Y_AXIS]) +
                                sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) { cartesian_mm = abs(difference[E_AXIS]); }
    if (cartesian_mm < 0.000001) { return; }
    float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
    int steps = max(1, int(scara_segments_per_second * seconds));

    //SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
    //SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
    //SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);

    for (int s = 1; s <= steps; s++) {
      float fraction = float(s) / float(steps);
      for(int8_t i = 0; i < NUM_AXIS; i++) {
        destination[i] = current_position[i] + difference[i] * fraction;
      }

      calculate_delta(destination);
      //SERIAL_ECHOPGM("destination[X_AXIS]="); SERIAL_ECHOLN(destination[X_AXIS]);
      //SERIAL_ECHOPGM("destination[Y_AXIS]="); SERIAL_ECHOLN(destination[Y_AXIS]);
      //SERIAL_ECHOPGM("destination[Z_AXIS]="); SERIAL_ECHOLN(destination[Z_AXIS]);
      //SERIAL_ECHOPGM("delta[X_AXIS]="); SERIAL_ECHOLN(delta[X_AXIS]);
      //SERIAL_ECHOPGM("delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
      //SERIAL_ECHOPGM("delta[Z_AXIS]="); SERIAL_ECHOLN(delta[Z_AXIS]);

      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS],
        destination[E_AXIS], feedrate*feedmultiply/60/100.0,
        active_extruder);
    }

  #endif // SCARA
#endif  

  #ifdef DELTA
    float difference[NUM_AXIS];
    for (int8_t i=0; i < NUM_AXIS; i++) difference[i] = destination[i] - current_position[i];

    float cartesian_mm = sqrt(sq(difference[X_AXIS]) +
                              sq(difference[Y_AXIS]) +
                              sq(difference[Z_AXIS]));
    if (cartesian_mm < 0.000001) cartesian_mm = abs(difference[E_AXIS]);
    if (cartesian_mm < 0.000001) return;
    float seconds = 6000 * cartesian_mm / feedrate / feedmultiply;
    int steps = max(1, int(DELTA_SEGMENTS_PER_SECOND * seconds));

    // SERIAL_ECHOPGM("mm="); SERIAL_ECHO(cartesian_mm);
    // SERIAL_ECHOPGM(" seconds="); SERIAL_ECHO(seconds);
    // SERIAL_ECHOPGM(" steps="); SERIAL_ECHOLN(steps);

    for (int s = 1; s <= steps; s++) {
      float fraction = float(s) / float(steps);
      for (int8_t i = 0; i < NUM_AXIS; i++) destination[i] = current_position[i] + difference[i] * fraction;

      calculate_delta(destination);
	  
      //SERIAL_ECHOPGM("destination[0]="); SERIAL_ECHOLN(destination[0]);
      //SERIAL_ECHOPGM("destination[1]="); SERIAL_ECHOLN(destination[1]);
      //SERIAL_ECHOPGM("destination[2]="); SERIAL_ECHOLN(destination[2]);
      //SERIAL_ECHOPGM("delta[X_AXIS]="); SERIAL_ECHOLN(delta[X_AXIS]);
      //SERIAL_ECHOPGM("delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
      //SERIAL_ECHOPGM("delta[Z_AXIS]="); SERIAL_ECHOLN(delta[Z_AXIS]);

      adjust_delta(destination);
	  
      plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder, active_driver);
    }

  #endif // DELTA

#if 0 //aven_0813
  #ifdef DUAL_X_CARRIAGE
    if (active_extruder_parked) {
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
        // move duplicate extruder into correct duplication position.
        plan_set_position(inactive_extruder_x_pos, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        plan_buffer_line(current_position[X_AXIS] + duplicate_extruder_x_offset, current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[X_AXIS], 1, active_driver);
        sync_plan_position();
        st_synchronize();
        extruder_duplication_enabled = true;
        active_extruder_parked = false;
      }
      else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) { // handle unparking of head
        if (current_position[E_AXIS] == destination[E_AXIS]) {
          // this is a travel move - skit it but keep track of current position (so that it can later
          // be used as start of first non-travel move)
          if (delayed_move_time != 0xFFFFFFFFUL) {
            set_current_to_destination();
            if (destination[Z_AXIS] > raised_parked_position[Z_AXIS])
              raised_parked_position[Z_AXIS] = destination[Z_AXIS];
            delayed_move_time = millis();
            return;
          }
        }
        delayed_move_time = 0;
        // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
        plan_buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], min(max_feedrate[X_AXIS],max_feedrate[Y_AXIS]), active_extruder, active_driver);
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], max_feedrate[Z_AXIS], active_extruder, active_driver);
        active_extruder_parked = false;
      }
    }
  #endif //DUAL_X_CARRIAGE
#endif
  

  #if !defined(DELTA) && !defined(SCARA)
    // Do not use feedmultiply for E or Z only moves
    if ((current_position[X_AXIS] == destination [X_AXIS]) && (current_position[Y_AXIS] == destination [Y_AXIS])) {
      line_to_destination();
    }
    else {
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], (feedrate/60)*(feedmultiply/100.0), active_extruder, active_driver);
    }
  #endif // !defined(DELTA) && !defined(SCARA)

#if 0 //aven_0813
  #ifdef IDLE_OOZING_PREVENT || EXTRUDER_RUNOUT_PREVENT
    axis_last_activity = millis();
    axis_is_moving = false;
  #endif
#endif

  set_current_to_destination();
}

void prepare_arc_move(char isclockwise) {
  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, feedrate*feedmultiply/60/100.0, r, isclockwise, active_extruder, active_driver);

  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  set_current_to_destination();
  refresh_cmd_timeout();
}

#if HAS_CONTROLLERFAN

unsigned long lastMotor = 0; // Last time a motor was turned on
unsigned long lastMotorCheck = 0; // Last time the state was checked

void controllerFan() {
  uint32_t ms = millis();
  if (ms >= lastMotorCheck + 2500) { // Not a time critical function, so we only check every 2500ms
    lastMotorCheck = ms;
    if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || soft_pwm_bed > 0
      || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
      #if EXTRUDERS > 1
        || E1_ENABLE_READ == E_ENABLE_ON
        #if HAS_X2_ENABLE
          || X2_ENABLE_READ == X_ENABLE_ON
        #endif
        #if EXTRUDERS > 2
          || E2_ENABLE_READ == E_ENABLE_ON
          #if EXTRUDERS > 3
            || E3_ENABLE_READ == E_ENABLE_ON
          #endif
        #endif
      #endif
    ) {
      lastMotor = ms; //... set time to NOW so the fan will turn on
    }
    uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;
    // allows digital or PWM fan output to be used (see M42 handling)
    digitalWrite(CONTROLLERFAN_PIN, speed);
    analogWrite(CONTROLLERFAN_PIN, speed);
  }
}
#endif

#ifdef SCARA
void calculate_SCARA_forward_Transform(float f_scara[3])
{
  // Perform forward kinematics, and place results in delta[3]
  // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
  
  float x_sin, x_cos, y_sin, y_cos;
  
    //SERIAL_ECHOPGM("f_delta x="); SERIAL_ECHO(f_scara[X_AXIS]);
    //SERIAL_ECHOPGM(" y="); SERIAL_ECHO(f_scara[Y_AXIS]);
  
    x_sin = sin(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
    x_cos = cos(f_scara[X_AXIS]/SCARA_RAD2DEG) * Linkage_1;
    y_sin = sin(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
    y_cos = cos(f_scara[Y_AXIS]/SCARA_RAD2DEG) * Linkage_2;
   
  //  SERIAL_ECHOPGM(" x_sin="); SERIAL_ECHO(x_sin);
  //  SERIAL_ECHOPGM(" x_cos="); SERIAL_ECHO(x_cos);
  //  SERIAL_ECHOPGM(" y_sin="); SERIAL_ECHO(y_sin);
  //  SERIAL_ECHOPGM(" y_cos="); SERIAL_ECHOLN(y_cos);
  
    delta[X_AXIS] = x_cos + y_cos + SCARA_offset_x;  //theta
    delta[Y_AXIS] = x_sin + y_sin + SCARA_offset_y;  //theta+phi
  
    //SERIAL_ECHOPGM(" delta[X_AXIS]="); SERIAL_ECHO(delta[X_AXIS]);
    //SERIAL_ECHOPGM(" delta[Y_AXIS]="); SERIAL_ECHOLN(delta[Y_AXIS]);
}

void calculate_delta(float cartesian[3]){
  //reverse kinematics.
  // Perform reversed kinematics, and place results in delta[3]
  // The maths and first version has been done by QHARLEY . Integrated into masterbranch 06/2014 and slightly restructured by Joachim Cerny in June 2014
  
  float SCARA_pos[2];
  static float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi; 
  
  SCARA_pos[X_AXIS] = cartesian[X_AXIS] * axis_scaling[X_AXIS] - SCARA_offset_x;  //Translate SCARA to standard X Y
  SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * axis_scaling[Y_AXIS] - SCARA_offset_y;  // With scaling factor.
  
  #if (Linkage_1 == Linkage_2)
    SCARA_C2 = ( ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) ) / (2 * (float)L1_2) ) - 1;
  #else
    SCARA_C2 =   ( sq(SCARA_pos[X_AXIS]) + sq(SCARA_pos[Y_AXIS]) - (float)L1_2 - (float)L2_2 ) / 45000; 
  #endif
  
  SCARA_S2 = sqrt( 1 - sq(SCARA_C2) );
  
  SCARA_K1 = Linkage_1 + Linkage_2 * SCARA_C2;
  SCARA_K2 = Linkage_2 * SCARA_S2;
  
  SCARA_theta = ( atan2(SCARA_pos[X_AXIS],SCARA_pos[Y_AXIS])-atan2(SCARA_K1, SCARA_K2) ) * -1;
  SCARA_psi   =   atan2(SCARA_S2,SCARA_C2);
  
  delta[X_AXIS] = SCARA_theta * SCARA_RAD2DEG;  // Multiply by 180/Pi  -  theta is support arm angle
  delta[Y_AXIS] = (SCARA_theta + SCARA_psi) * SCARA_RAD2DEG;  //       -  equal to sub arm angle (inverted motor)
  delta[Z_AXIS] = cartesian[Z_AXIS];
  
  /*
  SERIAL_ECHOPGM("cartesian x="); SERIAL_ECHO(cartesian[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(cartesian[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(cartesian[Z_AXIS]);
  
  SERIAL_ECHOPGM("scara x="); SERIAL_ECHO(SCARA_pos[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHOLN(SCARA_pos[Y_AXIS]);
  
  SERIAL_ECHOPGM("delta x="); SERIAL_ECHO(delta[X_AXIS]);
  SERIAL_ECHOPGM(" y="); SERIAL_ECHO(delta[Y_AXIS]);
  SERIAL_ECHOPGM(" z="); SERIAL_ECHOLN(delta[Z_AXIS]);
  
  SERIAL_ECHOPGM("C2="); SERIAL_ECHO(SCARA_C2);
  SERIAL_ECHOPGM(" S2="); SERIAL_ECHO(SCARA_S2);
  SERIAL_ECHOPGM(" Theta="); SERIAL_ECHO(SCARA_theta);
  SERIAL_ECHOPGM(" Psi="); SERIAL_ECHOLN(SCARA_psi);
  SERIAL_ECHOLN(" ");*/
}

#endif

#ifdef TEMP_STAT_LEDS
static bool blue_led = false;
static bool red_led = false;
static uint32_t stat_update = 0;

void handle_status_leds(void) {
  float max_temp = 0.0;
  if(millis() > stat_update) {
    stat_update += 500; // Update every 0.5s
    for (int8_t cur_extruder = 0; cur_extruder < EXTRUDERS; ++cur_extruder) {
       max_temp = max(max_temp, degHotend(cur_extruder));
       max_temp = max(max_temp, degTargetHotend(cur_extruder));
    }
    #if HAS_TEMP_BED
      max_temp = max(max_temp, degTargetBed());
      max_temp = max(max_temp, degBed());
    #endif
    if((max_temp > 55.0) && (red_led == false)) {
      digitalWrite(STAT_LED_RED, 1);
      digitalWrite(STAT_LED_BLUE, 0);
      red_led = true;
      blue_led = false;
    }
    if((max_temp < 54.0) && (blue_led == false)) {
      digitalWrite(STAT_LED_RED, 0);
      digitalWrite(STAT_LED_BLUE, 1);
      red_led = false;
      blue_led = true;
    }
  }
}
#endif

void enable_all_steppers() {
  enable_x();
  enable_y();
  enable_z();
  enable_e0();
  enable_e1();
  enable_e2();
  enable_e3();
}

void disable_all_steppers() {
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 *  - check oozing prevent
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

  
  //#if HAS_FILRUNOUT //aven_test0826
	  //if ((printing || card.sdprinting) && (READ(FILRUNOUT_PIN)^FIL_RUNOUT_INVERTING))

  //aven_0907
    if(filrunout_flag == 1)
    {
	  if ((READ(F0_STOP)^FIL_RUNOUT_INVERTING))
      {
        if(filrunout_info_count = 10)
        {
		  SerialUSB.println("filrunout");
          //filrunout();
          filrunout_info_count = 0;
        }
		filrunout_info_count++;
      }
  //#endif
    }

	else
	{
      filrunout_info_count = 0;
	}
//aven_0907

  if (buflen < BUFSIZE - 1) get_command();

  unsigned long ms = millis();

  if (max_inactive_time && ms > previous_millis_cmd + max_inactive_time) kill();

  if (stepper_inactive_time && ms > previous_millis_cmd + stepper_inactive_time
      && !ignore_stepper_queue && !blocks_queued())
    disable_all_steppers();

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && ms > chdkHigh + CHDK_DELAY) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif

  #if HAS_KILL
    
    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
       killCount++;
    else if (killCount > 0)
       killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) kill();
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 750;
    if (!READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        enquecommands_P(PSTR("G28"));
        LCD_ALERTMESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif
    
  #if HAS_CONTROLLERFAN
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #ifdef EXTRUDER_RUNOUT_PREVENT
    if (ms > previous_millis_cmd + EXTRUDER_RUNOUT_SECONDS * 1000)
    if (degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus;
      switch(active_extruder) {
        case 0:
          oldstatus = E0_ENABLE_READ;
          enable_e0();
          break;
        #if EXTRUDERS > 1
          case 1:
            oldstatus = E1_ENABLE_READ;
            enable_e1();
            break;
          #if EXTRUDERS > 2
            case 2:
              oldstatus = E2_ENABLE_READ;
              enable_e2();
              break;
            #if EXTRUDERS > 3
              case 3:
                oldstatus = E3_ENABLE_READ;
                enable_e3();
                break;
            #endif
          #endif
        #endif
      }
      float oldepos = current_position[E_AXIS], oldedes = destination[E_AXIS];
      plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS],
                      destination[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS],
                      EXTRUDER_RUNOUT_SPEED / 60. * EXTRUDER_RUNOUT_ESTEPS / axis_steps_per_unit[E_AXIS], active_extruder, active_driver);
      current_position[E_AXIS] = oldepos;
      destination[E_AXIS] = oldedes;
      plan_set_e_position(oldepos);
      previous_millis_cmd = ms; // refresh_cmd_timeout()
      st_synchronize();
      switch(active_extruder) {
        case 0:
          E0_ENABLE_WRITE(oldstatus);
          break;
        #if EXTRUDERS > 1
          case 1:
            E1_ENABLE_WRITE(oldstatus);
            break;
          #if EXTRUDERS > 2
            case 2:
              E2_ENABLE_WRITE(oldstatus);
              break;
            #if EXTRUDERS > 3
              case 3:
                E3_ENABLE_WRITE(oldstatus);
                break;
            #endif
          #endif
        #endif
      }
    }
  #endif

  #ifdef DUAL_X_CARRIAGE
    // handle delayed move timeout
    if (delayed_move_time && ms > delayed_move_time + 1000 && !Stopped) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_to_current();
      prepare_move();
    }
  #endif

  #ifdef IDLE_OOZING_PREVENT
    if (degHotend(active_extruder) > IDLE_OOZING_MINTEMP && !debugDryrun() && !axis_is_moving && idleoozing_enabled) {
      #ifdef FILAMENTCHANGEENABLE
        if (!filament_changing)
      #endif
      {
        if(degHotend(active_extruder) < IDLE_OOZING_MAXTEMP && degTargetHotend(active_extruder) < IDLE_OOZING_MINTEMP) {
          IDLE_OOZING_retract(false);
        }
        else if((millis() - axis_last_activity) >  IDLE_OOZING_SECONDS*1000) {
          IDLE_OOZING_retract(true);
        }
      }
    }
  #endif

  #ifdef TEMP_STAT_LEDS
    handle_status_leds();
  #endif

  check_axes_activity();
  manage_led();
}

void kill()
{
  cli(); // Stop interrupts
  disable_heater();

  disable_all_steppers();

  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif

  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);
  LCD_ALERTMESSAGEPGM(MSG_KILLED);
  
  // FMC small patch to update the LCD before ending
  sei();   // enable interrupts
  for (int i = 5; i--; lcd_update()) delay(200); // Wait a short time
  cli();   // disable interrupts
  suicide();
  while(1) { /* Intentionally left empty */ } // Wait for reset
}

//#if HAS_FILRUNOUT //aven_test0826
  void filrunout() {
    //if (filrunoutEnqued == false) {
      //filrunoutEnqued = true;
	  //gcode_X7();
      //enquecommand("M600");
    //}
  }
//#endif

void Stop()
{
  disable_heater();
  if(Stopped == false) {
    Stopped = true;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

bool IsStopped() { return Stopped; };

#ifdef FAST_PWM_FAN
  void setPwmFrequency(uint8_t pin, int val) {
    val &= 0x07;
    switch(digitalPinToTimer(pin)) {
      #if defined(TCCR0A)
      case TIMER0A:
      case TIMER0B:
  //         TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
  //         TCCR0B |= val;
           break;
      #endif

      #if defined(TCCR1A)
      case TIMER1A:
      case TIMER1B:
  //         TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  //         TCCR1B |= val;
           break;
      #endif

      #if defined(TCCR2)
      case TIMER2:
      case TIMER2:
           TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
           TCCR2 |= val;
           break;
      #endif

      #if defined(TCCR2A)
      case TIMER2A:
      case TIMER2B:
           TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
           TCCR2B |= val;
           break;
      #endif

      #if defined(TCCR3A)
      case TIMER3A:
      case TIMER3B:
      case TIMER3C:
           TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
           TCCR3B |= val;
           break;
      #endif

      #if defined(TCCR4A)
      case TIMER4A:
      case TIMER4B:
      case TIMER4C:
           TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
           TCCR4B |= val;
           break;
     #endif

      #if defined(TCCR5A)
      case TIMER5A:
      case TIMER5B:
      case TIMER5C:
           TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
           TCCR5B |= val;
           break;
     #endif

    }
  }
#endif //FAST_PWM_FAN

bool setTargetedHotend(int code) {
  target_extruder = active_extruder;
  if (code_seen('T')) {
    target_extruder = code_value_short();
    if (target_extruder >= EXTRUDERS) {
      SERIAL_ECHO_START;
      switch(code) {
        case 104:
          SERIAL_ECHO(MSG_M104_INVALID_EXTRUDER);
          break;
        case 105:
          SERIAL_ECHO(MSG_M105_INVALID_EXTRUDER);
          break;
        case 109:
          SERIAL_ECHO(MSG_M109_INVALID_EXTRUDER);
          break;
        case 218:
          SERIAL_ECHO(MSG_M218_INVALID_EXTRUDER);
          break;
        case 221:
          SERIAL_ECHO(MSG_M221_INVALID_EXTRUDER);
          break;
      }
      SERIAL_ECHOLN(target_extruder);
      return true;
    }
  }
  return false;
}


float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void calculate_volumetric_multipliers() {
  for (int i=0; i<EXTRUDERS; i++)
    volumetric_multiplier[i] = calculate_volumetric_multiplier(filament_size[i]);
}
