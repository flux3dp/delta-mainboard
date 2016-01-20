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
#include "manage_led.h"
#include "vector_3.h"
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
 float feedrate = 1500.0, next_feedrate, saved_feedrate;
static long gcode_N, Stopped_gcode_LastN = 0;
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


// Lifetime manage
unsigned long rpi_last_active = 0;
unsigned long rpi_wifi_active = 0;
bool rpi_io1_flag;
bool rpi_io2_flag;


//aven_0509
// FLUX defines
int pleds=0;

int motors=0;
int motorc=0;

struct GlobalVariable global = {
  0 //home_btn_press
};

struct FilamentDetect filament_detect = {false, 0};

const int led_pins[3] = {LED_P1, LED_P2, LED_P3};

struct LedStatus led_st = {
  'W',              // situational Prepare
  'D',              // Wifi Prepare
  0,                // last update
  0,                // god mode
  { LED_WAVE, LED_OFF, LED_OFF },        // mode LED_WAVE_POWER_ON
  {0.0008, 1, 1},   // param_a
  {0, 0, 0}         // param_b
};

struct PlayStatus play_st = {
  0, // enable_linecheck
  0, // stashed
  0, // next_no
  0, //laser pwm=0
};

float k_value= 0.03;
float u_value;

int G28_f = 0;

float cartesian[3] = { 0, 0, 0 };
float odelta[3] = { 0, 0, 0 };
float cdelta[3] = { 0, 0, 0 };

float h_offset=0;

// Hotend offset
#if HOTENDS > 1
  #ifndef DUAL_X_CARRIAGE
    #define NUM_HOTEND_OFFSETS 2 // only in XY plane
  #else
    #define NUM_HOTEND_OFFSETS 3 // supports offsets in XYZ plane
  #endif
  float hotend_offset[NUM_HOTEND_OFFSETS][HOTENDS];
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

static bool filrunoutEnqued = false;
bool printing = false;

bool filament_changing = false;

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


inline char get_pi_status() {
  if (rpi_io1_flag == (digitalRead(R_IO1) == HIGH)) {
    // rpi does not change R_IO1 status, check if timeout
    if (rpi_last_active) {
      if (millis() - rpi_last_active > 5000) {
        return PI_FATEL;
      }
      else {
        return PI_NOT_DEFINED;
      }
    } else {
      return PI_WAKINGUP;
    }
  }
  else {
    // rpi change R_IO1 flag, update status
    rpi_io1_flag = !rpi_io1_flag;
    rpi_last_active = millis();
    return PI_NOT_DEFINED;
  }
}

inline char get_wifi_status() {
   if(rpi_io2_flag == (digitalRead(R_IO2) == HIGH)) {
     if(millis() - rpi_wifi_active > 5000) {
       // rpi wifi status does not change over 5s
       if(rpi_io2_flag)  // wifi is up
         return PI_WIFI_CONNECTED;
       else if(!rpi_io2_flag)  // sleep
         return PI_WIFI_DISCONNECTED;
     } else {
       return PI_WIFI_ASSOCOATING;
     }
   } else {
     // rpi wifi status is changed, wave led
     rpi_io2_flag = !rpi_io2_flag;
     rpi_wifi_active = millis();
     return PI_WIFI_ASSOCOATING;
   }
}

inline char get_led_status() {
  if (play_st.enable_linecheck == 1) {  // if running
    switch(play_st.stashed) {
      case 0: return PI_RUNNING;
      case 1: return PI_PAUSED;
      default: return PI_ERROR;
    }
  } else {
    return PI_IDLE;
  }
}

inline void update_led_flags(char operation_flag, char wifi_flag) {
  if(operation_flag != 'W') {
    switch(wifi_flag) {
      case PI_WIFI_CONNECTED:
        if(led_st.mode[2] != LED_ON) led_st.mode[2] = LED_WAVE_2_ON;
        break;
      case PI_WIFI_ASSOCOATING:
        if(led_st.mode[2] != LED_WAVE) {
          led_st.mode[2] = LED_WAVE;
          led_st.param_a[2] = 0.0009;
          led_st.param_b[2] = millis();
        }
        break;
      case PI_WIFI_DISCONNECTED:
        led_st.mode[2] = LED_OFF;
        break;
    }
  }

	switch (operation_flag) {
	case PI_SLEEP: //Sleep
		led_st.mode[0] = led_st.mode[1] = led_st.mode[2] = LED_OFF;
		break;
	case PI_IDLE: //白燈呼吸燈 系統待機
		led_st.param_a[0] = 0.0004;
    led_st.param_b[0] = millis();
		led_st.mode[0] = LED_WAVE;
		led_st.mode[1] = LED_OFF;
		break;
	case PI_PAUSED: //白燈閃爍 工作暫停
		led_st.param_a[0] = 0.0015;
		led_st.param_b[0] = millis();
		led_st.mode[0] = LED_BLINK;
		led_st.mode[1] = LED_OFF;
		break;
	case PI_RUNNING: //白燈恆亮 工作中
		led_st.mode[0] = LED_ON;
		led_st.mode[1] = LED_OFF;
		break;
	case PI_FATEL: //橘燈恆亮 系統故障
		led_st.mode[0] = LED_OFF;
		led_st.mode[1] = LED_ON;
		break;
	case PI_ERROR: //橘燈閃爍 工作異常
		led_st.mode[0] = LED_OFF;
		led_st.param_a[1] = 0.0015;
		led_st.param_b[1] = millis();
		led_st.mode[1] = LED_BLINK;
		break;
	case PI_RUNNING_WAIT_HEAD: //橘燈呼吸燈 準備中
    led_st.mode[0] = LED_OFF;
    led_st.param_a[1] = 0.0004;
    led_st.param_b[1] = millis();
    led_st.mode[1] = LED_WAVE;
		break;
  case PI_UPDATE: //更新中
		led_st.param_a[0] = 0.0015;
		led_st.param_b[0] = 666.6666;
		led_st.mode[0] = LED_BLINK;
		led_st.param_a[1] = 0.0015;
		led_st.param_b[1] = 0;
		led_st.mode[1] = LED_BLINK;
		break;
  case PI_STARTING_TASK:
    led_st.param_a[0] = 0.003;
    led_st.param_b[0] = millis();
    led_st.mode[0] = LED_BLINK;
    led_st.mode[1] = LED_OFF;
    break;
  case PI_WAKINGUP:
    led_st.param_a[0] = 0.0008;
    led_st.mode[0] = LED_WAVE;
    led_st.mode[1] = LED_OFF;
    break;
	default:
  	led_st.mode[0] = LED_OFF;
  	led_st.mode[1] = LED_OFF;
  	led_st.mode[2] = LED_BLINK;
		led_st.param_a[2] = 0.01;
		led_st.param_b[2] = millis();
	}
}

void manage_led()
{
  if (millis() - led_st.last_update < 30) return;
  led_st.last_update = millis();

  char new_situational = get_pi_status();
  char new_wifi_flag = get_wifi_status();

  if (new_situational == PI_NOT_DEFINED) {
    if(new_wifi_flag == PI_WIFI_DISCONNECTED) {
      new_situational = PI_SLEEP;
    } else {
      new_situational = get_led_status();
    }
  }

  if (new_situational != 'F' && led_st.god_mode) {
    new_situational = led_st.god_mode;
  }

  if(new_situational != led_st.situational || new_wifi_flag != led_st.wifi) {

    update_led_flags(new_situational, new_wifi_flag);
    led_st.situational = new_situational;
    led_st.wifi = new_wifi_flag;
  }

	for (int i = 0; i<3; i++) {
		switch (led_st.mode[i]) {
		case LED_OFF:
			analogWrite(led_pins[i], 0);
			break;
		case LED_WAVE:
			analogWrite(led_pins[i], int(_led_wave(i) * 255));
			break;
		case LED_BLINK:
			analogWrite(led_pins[i], (_led_blink(i) > 0.5) ? 255 : 0);
			break;
		case LED_ON:
			analogWrite(led_pins[i], 255);
			break;
		case LED_WAVE_2_ON:
			if (_led_wave(i) > 0.95) {
				analogWrite(led_pins[i], 255);
				led_st.mode[i] = LED_ON;
			}
			else {
				analogWrite(led_pins[i], int(_led_wave(i) * 255));
			}
			break;
		case LED_WAVE_2_OFF:
			if (_led_wave(i) < 0.05) {
				analogWrite(led_pins[i], 0);
				led_st.mode[i] = LED_OFF;
			}
			else {
				analogWrite(led_pins[i], int(_led_wave(i) * 255));
			}
			break;
		case LED_STATIC:
			analogWrite(led_pins[i], int(led_st.param_a[i]));
			break;
		default:
			analogWrite(led_pins[i], 0);
			break;
		}
	}
}


void on_home_btn_press() {
  global.home_btn_press += 1;
}

bool inline enter_boot_mode() {
  int counter = 0;
  while(led_st.situational == 'W' && !digitalRead(HOME_BTN_PIN)) {
    counter += 1;
    manage_inactivity();
    delay(10);
    if(counter > 500) return true;
  }
  return false;
}

void setup()
{
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

  //tp_init();    // Initialize temperature loop
  plan_init();  // Initialize planner;
  watchdog_init();
  st_init();    // Initialize stepper, this enables interrupts!

  setup_homepin();

  pinMode(S_LAS1, OUTPUT);//PC25
  pinMode(S_LAS2, OUTPUT);//PC26
  pinMode(U5EN, OUTPUT);
  pinMode(U5FAULT, INPUT_PULLUP);
  digitalWrite(U5EN, LOW);
  pinMode(M_IO1, INPUT_PULLUP);//PD7
  pinMode(M_IO2,OUTPUT); //PD8 Laser PWM
  digitalWrite(M_IO2, 0);
  pinMode(CAP_IO,INPUT);
  pinMode(REF_IO,INPUT);

  //// Initial LED
  pinMode(LED_P1,OUTPUT);
  pinMode(LED_P2,OUTPUT);
  pinMode(LED_P3,OUTPUT);

  analogWrite(LED_P1, 255);
  analogWrite(LED_P2, 255);
  analogWrite(LED_P3, 255);

  digitalWrite(S_LAS1, LOW);
  digitalWrite(S_LAS2, LOW);

  pinMode(HOME_K,INPUT);

  // LED Control
  pinMode(R_IO1,INPUT);
  pinMode(R_IO2,INPUT);
  rpi_io1_flag = digitalRead(R_IO1) == HIGH;
  rpi_io2_flag = digitalRead(R_IO2) == HIGH;
  led_st.param_b[0] = millis();

  pinMode(REFC1,OUTPUT); 
  pinMode(REFC2,OUTPUT);
  pinMode(REFC3,OUTPUT); 
  pinMode(REFC4,OUTPUT);

  digitalWrite(REFC1, LOW);
  digitalWrite(REFC2, LOW);
  digitalWrite(REFC3, LOW);
  digitalWrite(REFC4, LOW);

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

  pinMode(F0_STOP, INPUT);

  pinMode(HOME_BTN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HOME_BTN_PIN),on_home_btn_press, FALLING);
  analogReadResolution(12);
}

void inline report_ln() {
  SERIAL_PROTOCOL("LN ");
  SERIAL_PROTOCOL(play_st.last_no);
  SERIAL_PROTOCOL(" ");
  SERIAL_PROTOCOL(buflen);
  SERIAL_PROTOCOL("\n");
}


void loop() {
  if (buflen < BUFSIZE - 1) get_command();

  if (buflen) {
    bool ret = process_commands();

    if(!ret) {
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
      SERIAL_ECHO(cmdbuffer[bufindr]);
      SERIAL_ECHOLNPGM("\"");

      SERIAL_PROTOCOLLN("ER BAD_CMD");
    }

    buflen--;
    bufindr = (bufindr + 1) % BUFSIZE;

    if(play_st.enable_linecheck) {
      report_ln();
    } else if(ret) {
      if(!play_st.enable_linecheck) {
        SERIAL_PROTOCOLLN(MSG_OK);
      }
    }
  }
  // Check heater every n milliseconds
  manage_heater();
  manage_inactivity();
  checkHitEndstops();
  manage_led();

  if(led_st.situational == 'W' && enter_boot_mode()) {
    led_st.god_mode = 'U';
  }
}

void inline proc_heigh_level_control(const char* cmd) {
  if(strcmp(cmd, "ENABLE_LINECHECK") == 0) {
    SERIAL_PROTOCOLLN("CTRL LINECHECK_ENABLED");
    play_st.enable_linecheck = 1;
    play_st.last_no = 0;

  } else if(strcmp(cmd, "DISABLE_LINECHECK") == 0) {
    while(buflen > 1) {
      buflen--;
      bufindr = (bufindr + 1) % BUFSIZE;
    }

    SERIAL_PROTOCOLLN("CTRL LINECHECK_DISABLED");
    play_st.enable_linecheck = 0;
    play_st.stashed = 0;

  } else if(strcmp(cmd, "HOME_BUTTON_TRIGGER") == 0) {
    global.home_btn_press++;

  } else if(strcmp(cmd, "OOPS") == 0) {
    report_ln();

  } else {
    SERIAL_PROTOCOLLN("ER UNKNOW_CMD");
  }
}


bool inline check_line_number(const char* cmd) {
  // Check N
  strchr_pointer = strchr(cmd, 'N');

  if(strchr_pointer == NULL) {
    // Can not found Line Number, send error, clean buffer and return
    SERIAL_PROTOCOL("ER MISSING_LINENUMBER ");
    SERIAL_PROTOCOL(play_st.last_no + 1);
    SERIAL_PROTOCOL("\n");
    MYSERIAL.flush();
    return false;
  } else {
    gcode_N = (strtol(strchr_pointer + 1, NULL, 10));
    if(gcode_N != play_st.last_no + 1) {
      SERIAL_PROTOCOL("ER LINE_MISMATCH ");
      SERIAL_PROTOCOL(play_st.last_no + 1);
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
    SERIAL_PROTOCOL(play_st.last_no + 1);
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
      SERIAL_PROTOCOL(play_st.last_no + 1);
      SERIAL_PROTOCOL("\n");
      MYSERIAL.flush();
      return false;
    }
  }

  play_st.last_no = gcode_N;
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

      if(cmdbuffer[bufindw][0] == '@') {
        proc_heigh_level_control(cmdbuffer[bufindw] + 1);
        serial_count = 0;
        return;
      }

      //If command was e-stop process now
      if(strcmp(cmdbuffer[bufindw], "M112") == 0)
        kill();

      if(play_st.enable_linecheck == 1) {
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

      if(play_st.enable_linecheck) {
        report_ln();
      }
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


inline void read_fsr_helper(int times, float avg[3], float sd[3],
                            int max_value[3], int min_value[3]) {
    int dataset[3][200];
    for(int i=0;i<3;i++) {
        max_value[i] = min_value[i] = dataset[i][0] = analogRead(i);
    }
    for(int x=1;x<times;x++) {
        for(int i=0;i<3;i++) {
            int val = dataset[i][x] = analogRead(i);
            if(val > max_value[i]) max_value[i] = val;
            if(val < min_value[i]) min_value[i] = val;
        }
    }
    for(int i=0;i<3;i++) {
        long sum = 0;
        for(int x=0;x<times;x++) sum += dataset[i][x];
        avg[i] = (float)sum / (float)times;
        float v = 0;
        for(int x=1;x<times;x++) v += pow(avg[i] - dataset[i][x], 2);
        sd[i] = pow(v / (float)(times - 1), 0.5);
    }
}


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
      destination[axis] = 3 * max_length[axis] * axis_home_dir;
      feedrate = homing_feedrate[axis];
      line_to_destination();
      st_synchronize();

	  switch (axis) {
	  case 0:
		  endstop_has_hit[axis] = endstop_x_hit;
		  break;
	  case 1:
		  endstop_has_hit[axis] = endstop_y_hit;
		  break;
	  case 2:
		  endstop_has_hit[axis] = endstop_z_hit;
		  break;

	  }
	  
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



  int read_FSR(float &data, int count, float ratio[])
  {
    float avg[3], sd[3];
    int max_val[3], min_val[3];

    read_fsr_helper(count, avg, sd, max_val, min_val);

    bool flag = 1;

    data = 0;
    for (int j = 0; j < 3; j++)
    {
      data += ratio[j] * avg[j];
      if (sd[j] * 3 > avg[j] * 0.01)
        flag = 0;
    }
    return flag;
  }


  float z_probe()
  {
    feedrate = homing_feedrate[X_AXIS];
    
    prepare_move_raw();
    st_synchronize();

    float start_z = current_position[Z_AXIS];
    long start_steps = st_get_position(Z_AXIS);

    feedrate = probing_feedrate;

    float ratio[3];
    const float point[3][2] = {{-73.6122, -42.5}, {73.6122, -42.5}, {0, 85}};

    for (int i = 0; i < 3; i++)
    {
      ratio[(i + 2) % 3] = point[i][0] * point[(i + 1) % 3][1] - point[i][1] * point[(i + 1) % 3][0];
      ratio[(i + 2) % 3] += point[(i + 1) % 3][0] * destination[Y_AXIS] - point[(i + 1) % 3][1] * destination[X_AXIS];
      ratio[(i + 2) % 3] += destination[X_AXIS] * point[i][1] - destination[Y_AXIS] * point[i][0];
    }

    float data;
    
    read_FSR(data, 20, ratio);
    float threshold = data;
    int fsr_flag = -1;
	int fsr_result;
	//downward
    while ((destination[Z_AXIS] -= 0.00625) > (MANUAL_Z_HOME_POS-243.5)  && fsr_flag < 0)
    {
      fsr_flag--;
      prepare_move_raw();
      st_synchronize();
      
	  fsr_result = read_FSR(data, 20, ratio);
	  //Start detecting metal plate existed by Shuo 12/11
	  if (fsr_result < -150)
		  return fsr_result;
	  //end
      if (fsr_flag > -500)
      {
        if (data < threshold)
        {
          threshold = data;
          
        }
      }
      else
      {
        if (data < threshold * 0.998)
        {
          fsr_flag = 1;
        } 
        else
          delayMicroseconds(200);
      }
	
      
    }
	if (destination[Z_AXIS] < (MANUAL_Z_HOME_POS - 243.5))
		return -300;

    float z_val = destination[Z_AXIS];
    prepare_move_raw();
    st_synchronize();

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

    
    feedrate = 600;

    z_val += 0.2;
    float up, down;
    float value[3] = {0};
	
    for (int i = 0; i < 30; i++)
    {
      destination[Z_AXIS] = z_val + 0.25;
      prepare_move_raw();
      st_synchronize();
      delay(200);
      
      int timeout = 0;
	  while (!(fsr_result = read_FSR(up, 200, ratio)) && timeout < 20) {
		  if (fsr_result < -150)
			  return fsr_result;
		  timeout++;
	  }
      destination[Z_AXIS] = (z_val -= 0.0125) ;
      prepare_move_raw();
      st_synchronize();
      delay(200);
      
      timeout = 0;
	  while (!(fsr_result = read_FSR(down, 200, ratio)) && timeout < 20) {
		  if (fsr_result < -150)
			  return fsr_result;
		  timeout++;
	  }

      for (int j = 2; j > 0; j--)
        value[j] = value[j - 1];
      value[0] = up - down;
      if (value[0] < up * 0.005)
        value[0] = 1;
      if (value[2] == 0) 
      {
        if (value[0] > 1)
        {
          z_val += 0.1;
          value[1] = 0;
        }
      }
      else if (value[2] > 1)
      {
        if (value[0] * 0.99 > value[1] && value[1] * 0.99 > value[2])
        {
          destination[Z_AXIS] += 1;
          prepare_move_raw();
          st_synchronize();
          return z_val + 0.0375;
        }
        else
          return -100;
      }
      
    }
    
    
    return -100;
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
    feedrate = homing_feedrate[Z_AXIS];
    destination[X_AXIS] = x - z_probe_offset[X_AXIS];
    if (destination[X_AXIS]<X_MIN_POS) destination[X_AXIS]=X_MIN_POS;
    if (destination[X_AXIS]>X_MAX_POS) destination[X_AXIS]=X_MAX_POS;
    destination[Y_AXIS] = y - z_probe_offset[Y_AXIS];
    if (destination[Y_AXIS]<Y_MIN_POS) destination[Y_AXIS]=Y_MIN_POS;
    if (destination[Y_AXIS]>Y_MAX_POS) destination[Y_AXIS]=Y_MAX_POS;

	float data;
	float ratio[3];
	//Start detecting metal plate existed by Shuo 12/11
	int fsr_result;
	fsr_result=read_FSR(data, 20, ratio);
	if (fsr_result <= -200) {
		return fsr_result;
	}
	//End
    destination[Z_AXIS] = 6;
    prepare_move();
    st_synchronize();
    return z_probe() + z_probe_offset[Z_AXIS];
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

  }

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
      }
      else {
        x_done = false;
		SerialUSB.println("X=ERROR ");
      }

      if ((bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)) {
        y_done = true;
		SerialUSB.println("Y=OK ");
      }
      else {
        y_done = false;
		SerialUSB.println("Y=ERROR ");
      }

      if ((bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)) {
        z_done = true;
		SerialUSB.println("Z=OK ");
      }
      else {
        z_done = false;
		SerialUSB.println("Z=ERROR");
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
  
  }

#endif //DELTA


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
    get_coordinates(); // For X Y Z E F
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
	if (code_seen('+')) {
		G28_f = 1;
	}
	else {
		G28_f = 0;
	}
  
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

#ifdef ENDSTOPS_ONLY_FOR_HOMING
	enable_endstops(false);
#endif

	if (G28_f && READ(M_IO1) == LOW) {
		G28_f = 0;
		refresh_cmd_timeout();

		//SERIAL_PROTOCOLLN("detected alarmIO");
		enable_endstops(true);

		// Move all carriages up together until the first endstop is hit.
		for (int i = X_AXIS; i <= Z_AXIS; i++) current_position[i] = 0;
		sync_plan_position();

		// Move all carriages up together until the first endstop is hit.
		for (int i = X_AXIS; i <= Z_AXIS; i++) destination[i] = -0.5;
		feedrate = 1.732 * homing_feedrate[X_AXIS];
		line_to_destination();
		st_synchronize();

		endstops_hit_on_purpose(); // clear endstop hit flags
		SERIAL_PROTOCOLLN("ER G28_FAILED");
	}else if(!(endstop_has_hit[0] && endstop_has_hit[1] && endstop_has_hit[2])) {
		endstop_has_hit[0] = endstop_has_hit[1] = endstop_has_hit[2] = false;
		SERIAL_PROTOCOLLN("ER G28_FAILED");
	}

	

  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  refresh_cmd_timeout();
  endstops_hit_on_purpose(); // clear endstop hit flags
  G28_f = 0; //aven_0807

}

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

      //deploy_z_probe();
      int count = 0;
      do
      {
        probe_value = probe_bed(x, y);
        count++;
      } while (probe_value < -10 && probe_value > -150 && count < 3);
      SERIAL_ECHO("Bed Z-Height at X:");
      SERIAL_ECHO(x);
      SERIAL_ECHO(" Y:");
      SERIAL_ECHO(y);
      SERIAL_ECHO(" = ");
      /*
      // NOTE: Change probe_value because FSR is obtuse in center
      float d = pow(pow(x, 2) + pow(y, 2), 0.5);
      if(d < 85) {
          probe_value += 0.3 * ((85 - d) / 85);  // Linear adjust
      }
      */

      SERIAL_PROTOCOL_F(probe_value, 4);
	  SERIAL_EOL;
	  SERIAL_ECHO("DEBUG: Retry= ");
	  SERIAL_ECHO(count);
      SERIAL_EOL;

      SERIAL_ECHO("Carriage Positions: [");
      SERIAL_ECHO(saved_position[X_AXIS]);
      SERIAL_ECHO(", ");
      SERIAL_ECHO(saved_position[Y_AXIS]);
      SERIAL_ECHO(", ");
      SERIAL_ECHO(saved_position[Z_AXIS]);
      SERIAL_ECHOLN("]");
      //retract_z_probe();
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
  if (didXYZ) sync_plan_position_delta();
}

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
  SERIAL_PROTOCOLPGM("CTRL STATUS X:");
  SERIAL_PROTOCOL(current_position[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(current_position[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(current_position[Z_AXIS]);
  SERIAL_PROTOCOLPGM(" E:");
  SERIAL_PROTOCOL(current_position[E_AXIS]);
  SERIAL_PROTOCOLPGM(" T:");
  SERIAL_PROTOCOL(active_extruder);

  SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  SERIAL_PROTOCOL(float(st_get_position(X_AXIS))/axis_steps_per_unit[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(float(st_get_position(Y_AXIS))/axis_steps_per_unit[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(float(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS]);

  SERIAL_EOL;
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
    SERIAL_PROTOCOLLN(((!READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_X_MAX
    SERIAL_PROTOCOLPGM(MSG_X_MAX);
    SERIAL_PROTOCOLLN(((!READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MIN
    SERIAL_PROTOCOLPGM(MSG_Y_MIN);
    SERIAL_PROTOCOLLN(((!READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Y_MAX
    SERIAL_PROTOCOLPGM(MSG_Y_MAX);
    SERIAL_PROTOCOLLN(((!READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MIN
    SERIAL_PROTOCOLPGM(MSG_Z_MIN);
    SERIAL_PROTOCOLLN(((!READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_MAX
    SERIAL_PROTOCOLPGM(MSG_Z_MAX);
    SERIAL_PROTOCOLLN(((!READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z2_MAX
    SERIAL_PROTOCOLPGM(MSG_Z2_MAX);
    SERIAL_PROTOCOLLN(((!READ(Z2_MAX_PIN)^Z2_MAX_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_Z_PROBE
    SERIAL_PROTOCOLPGM(MSG_Z_PROBE);
    SERIAL_PROTOCOLLN(((!READ(Z_PROBE_PIN)^Z_PROBE_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  #if HAS_E_MIN
    SERIAL_PROTOCOLPGM(MSG_E_MIN);
    SERIAL_PROTOCOLLN(((!READ(E_MIN_PIN)^E_MIN_ENDSTOP_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
  #endif
  //#if HAS_FILRUNOUT //aven_test0826
  //aven_0817 FILAMENT RUNOUT
    //SERIAL_PROTOCOLPGM(MSG_FILRUNOUT_PIN);
    SERIAL_PROTOCOLPGM(MSG_FILAMENT_RUNOUT_PIN);
    SERIAL_PROTOCOLLN(((!READ(F0_STOP)^FIL_RUNOUT_INVERTING)?MSG_ENDSTOP_HIT:MSG_ENDSTOP_OPEN));
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

/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
  if (debugDryrun()) return;
  if (code_seen('S')) setTargetBed(code_value());
}

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
}

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

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { st_synchronize(); }

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

#ifdef DELTA
  //M666: Set delta endstop and geometry adjustment
  inline void gcode_M666() {
    if ( !(code_seen('P'))) {
      float saved_endstop_adj[4] = {0};
      for(int8_t i=0; i < 3; i++) 
      {
        if (code_seen(axis_codes[i])) 
        {
          saved_endstop_adj[i] = code_value() - endstop_adj[i];
          endstop_adj[i] = code_value();
        }
      }
      if (code_seen('H'))
      {
        saved_endstop_adj[3] = code_value() - max_pos[Z_AXIS];
        max_pos[Z_AXIS]= code_value();
        set_delta_constants();
      }
      calculate_delta(current_position);
      plan_set_position(delta[X_AXIS] - saved_endstop_adj[X_AXIS] + saved_endstop_adj[3], delta[Y_AXIS] - saved_endstop_adj[Y_AXIS] + saved_endstop_adj[3], delta[Z_AXIS] - saved_endstop_adj[Z_AXIS] + saved_endstop_adj[3], current_position[E_AXIS]);
      st_synchronize();
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

/**
 * M999: Restart after being stopped
 */
inline void gcode_M999() {
  Stopped = false;
  lcd_reset_alert_level();
  play_st.last_no = Stopped_gcode_LastN;
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
            //SERIAL_ECHO_START;
            //SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
            //SERIAL_PROTOCOLLN((int)active_extruder);

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

	  SERIAL_ECHO_START;
	  SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
	  SERIAL_PROTOCOLLN((int)active_extruder);

      #ifdef EXT_SOLENOID
        st_synchronize();
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

    #endif // EXTRUDERS > 1
  }
}

inline void gcode_X1()
{
  if (code_seen('E')) {
    pleds = code_value_short();
    digitalWrite(S_LAS1, (pleds & 1) ? HIGH : LOW);
    digitalWrite(S_LAS2, (pleds & 2) ? HIGH : LOW);
    return;
  }

  // TODO: This codes is goning to be removed
  if (code_seen('F')) 
  {
    pleds = code_value_short();
    if(pleds == 1)
    {
      digitalWrite(S_LAS1, LOW);
    }
    if(pleds == 2)
    {
      digitalWrite(S_LAS2, LOW);
    }

    if(pleds == 0)
    {
      digitalWrite(S_LAS1, LOW);
      digitalWrite(S_LAS2, LOW);
    }
  }
  
  if (code_seen('O')) 
  {
    pleds = code_value_short();
    if(pleds == 1)
    {
      digitalWrite(S_LAS1, HIGH);
    }
    if(pleds == 2)
    {
      digitalWrite(S_LAS2, HIGH);
    }
    if(pleds == 0)
    {
      digitalWrite(S_LAS1, HIGH);
      digitalWrite(S_LAS2, HIGH);
    }
  }
}

inline void gcode_X2()
{
  st_synchronize();
  if (code_seen('F'))
  {
	  play_st.stashed_laser_pwm = 0;
    analogWrite(M_IO2, 0);
  }

  if (code_seen('O'))
  {
    pleds = code_value_short();
    if(pleds >= 0 & pleds <= 255)
    {
		play_st.stashed_laser_pwm = pleds;
      analogWrite(M_IO2, pleds);
    }
  }
}

inline void gcode_X3()
{
  if (code_seen('F')) 
  {
    digitalWrite(REFC1, LOW);
    digitalWrite(REFC2, LOW);
    digitalWrite(REFC3, LOW);
    digitalWrite(REFC4, LOW);
  }
  if (code_seen('O')) 
  {
    digitalWrite(REFC1, HIGH);
    digitalWrite(REFC2, HIGH);
    digitalWrite(REFC3, HIGH);
    digitalWrite(REFC4, HIGH);
  }
}

inline void gcode_X4()
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
    SerialUSB.print((int)led_st.mode[led_index]);
    SerialUSB.print(" PARAM_A ");
    SerialUSB.print(led_st.param_a[led_index]);
    SerialUSB.print(" PARAM_B ");
    SerialUSB.print(led_st.param_a[led_index]);
    return;
  }
  if(code_seen('C')) {
    led_st.mode[led_index] = code_value();
    SerialUSB.print(" CONTROL ");
    SerialUSB.print(led_st.mode[led_index]);
  }
  if(code_seen('A')) {
    led_st.param_a[led_index] = code_value();
    SerialUSB.print(" PARAM_A ");
    SerialUSB.print(led_st.param_a[led_index]);
  }
  if(code_seen('B')) {
    led_st.param_b[led_index] = code_value();
    SerialUSB.print(" PARAM_B ");
    SerialUSB.print(led_st.param_b[led_index]);
  }
  SerialUSB.println("#");
}

inline void gcode_X5() {
  if(code_seen('S')) {
    led_st.god_mode = code_value_short();
  } else {
    SerialUSB.print("INFO: ST=");
    SerialUSB.println(led_st.situational);
  }
}


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
    play_st.enable_linecheck = 1;
    play_st.last_no = 0;
    play_st.stashed = 0;
    led_st.god_mode = 0;
  } else if (code_seen('F')) {
    SERIAL_PROTOCOLLN("CTRL LINECHECK_DISABLED");
    play_st.enable_linecheck = 0;
    play_st.stashed = 0;
    led_st.god_mode = 0;
  } else {
    SERIAL_PROTOCOLLN("ER BAD_CMD");
  }
}

inline void gcode_C2()
{
  if (play_st.enable_linecheck == 0) {
    SERIAL_PROTOCOLLN("ER LINECHECK_DISABLED");
    return;
  }

  if (code_seen('O') || code_seen('E'))
  {
    if (play_st.stashed == 0) {
      play_st.stashed = code_seen('E') ? code_value_short() : 1;
	  st_synchronize();
	  analogWrite(M_IO2, 0);

      // Remember current status
      play_st.stashed_position[X_AXIS] = current_position[X_AXIS];
      play_st.stashed_position[Y_AXIS] = current_position[Y_AXIS];
      play_st.stashed_position[Z_AXIS] = current_position[Z_AXIS];
      for(int i=Z_AXIS + 1;i<NUM_AXIS;i++) {
        play_st.stashed_extruder_position[i] = current_position[i];
      }
      play_st.stashed_feedrate = feedrate;
      play_st.stashed_extruder = target_extruder;

      // Retraction
      feedrate = 500;
      destination[E_AXIS] = current_position[E_AXIS] - 5;
      prepare_move();
      st_synchronize();

      if (current_position[Z_AXIS] < 210) {
        destination[Z_AXIS] = min(current_position[Z_AXIS] + 5, 210);
        feedrate = 300;
        prepare_move_raw();
        st_synchronize();
      }
      if (current_position[Z_AXIS] < 210) {
        destination[Z_AXIS] = min(current_position[Z_AXIS] + 25, 210);
        feedrate = 2500;
        prepare_move_raw();
        st_synchronize();
      }

      SERIAL_PROTOCOLLN("CTRL STASH");


    } else {
      SERIAL_PROTOCOLLN("ER ALREADY_STASHED");
    }
  } else if (code_seen('F')) {
    if (play_st.stashed == 0) {
      SERIAL_PROTOCOLLN("ER NOT_STASHED");
    } else {
		
		feedrate = 6000;
	  destination[X_AXIS] = current_position[X_AXIS];
	  destination[Y_AXIS] = current_position[Y_AXIS];
      destination[Z_AXIS] = play_st.stashed_position[Z_AXIS];
	  destination[E_AXIS] = current_position[E_AXIS];
      prepare_move_raw();
      st_synchronize();

	  analogWrite(M_IO2, play_st.stashed_laser_pwm);
      feedrate = play_st.stashed_feedrate;
      target_extruder = play_st.stashed_extruder;

      play_st.stashed = 0;
      SERIAL_PROTOCOLLN("CTRL STASH_POP");
    }
  } else {
    SERIAL_PROTOCOLLN("ER BAD_CMD");
  }
}
inline void gcode_C3(int t=0) {
  if(t > 2) t = 0;
  target_extruder = t;

  float e_pos = current_position[E_AXIS];
  destination[X_AXIS] = current_position[X_AXIS];
  destination[Y_AXIS] = current_position[Y_AXIS];
  destination[Z_AXIS] = current_position[Z_AXIS];
  destination[E_AXIS] = current_position[E_AXIS];
  // Move to stash position
  feedrate = 8000;
  if(current_position[Z_AXIS] > 200) {
    destination[Z_AXIS] = 220;
    prepare_move_raw();
    st_synchronize();
  }

  destination[X_AXIS] = 0;
  destination[Y_AXIS] = -90;
  destination[Z_AXIS] = 220;
  destination[E_AXIS] = current_position[E_AXIS];
  prepare_move_raw();
  st_synchronize();


  unsigned long timer = 0;
  float avg[3], sd[3];
  int dummy1[3], dummy2[3];
  read_fsr_helper(5, avg, sd, dummy1, dummy2);

  int max_value_base = avg[0] + sd[0] * 3;
  int speed = 150;
  bool check_filament = code_seen('+');

  global.home_btn_press = 0;
  while(global.home_btn_press == 0) {
    if(millis() - timer > 500) {
      if(READ(F0_STOP)^FIL_RUNOUT_INVERTING) {
        SERIAL_PROTOCOLLN("CTRL FILAMENT-");
      } else {
        SERIAL_PROTOCOLLN("CTRL FILAMENT+");
      }
      timer = millis();
    }

    if(check_filament && READ(F0_STOP)^FIL_RUNOUT_INVERTING) {
        delay(10);
        manage_inactivity();
        continue;
    }

    read_fsr_helper(5, avg, sd, dummy1, dummy2);

    if(avg[0] > max_value_base) max_value_base = avg[0];
    if(max_value_base > 3500) max_value_base = 3500;

    int new_speed = (max_value_base - avg[0]) * 6;
    if(new_speed > 6000) new_speed = 6000;
    new_speed = (new_speed / 500) * 500 + 150;

    if(new_speed - speed > 350) speed += 350;
    else if(new_speed - speed < -700) speed -= 700;
    else speed = new_speed;

    if (speed < 150) {
        speed = 150;
    }
    feedrate = speed;
    destination[E_AXIS] = current_position[E_AXIS] + (speed / 1000.0);

    prepare_move();
  }

  current_position[E_AXIS] = e_pos;
  plan_set_e_position(e_pos);
  SERIAL_PROTOCOLLN("ok");
}

inline void gcode_C4(int t=0) {
  if(t > 2) t = 0;
  target_extruder = t;

  destination[X_AXIS] = current_position[X_AXIS];
  destination[Y_AXIS] = current_position[Y_AXIS];
  destination[Z_AXIS] = current_position[Z_AXIS];
  float e_pos = current_position[E_AXIS];

  if(READ(F0_STOP)^FIL_RUNOUT_INVERTING) {
    SERIAL_PROTOCOLLN("ok");
    return;
  }

  feedrate = 8000;
  destination[E_AXIS] = current_position[E_AXIS] - 50;
  prepare_move();

  feedrate = 6000;
  for(int i=0;i<9;i++) {
    if(READ(F0_STOP)^FIL_RUNOUT_INVERTING) {
      SERIAL_PROTOCOLLN("ok");
      break;
    }

    destination[E_AXIS] -= 50;
    prepare_move();
  }

  current_position[E_AXIS] = e_pos;
  plan_set_e_position(e_pos);
  SERIAL_PROTOCOLLN("ok");
  return;
}

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

  }

  //cdelta[0] = (cdelta[0]/80) - 403.23;
  //cdelta[1] = (cdelta[1]/80) - 403.23;
  //cdelta[2] = (cdelta[2]/80) - 403.23;
  enable_endstops(false);

  cdelta[0] = h_constant - cdelta[0] - endstop_adj[0];
  cdelta[1] = h_constant - cdelta[1] - endstop_adj[1];
  cdelta[2] = h_constant - cdelta[2] - endstop_adj[2];

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

inline void gcode_X8()
{
  if (code_seen('O')) 
  {
    filament_detect.enable = true;
  }

  if (code_seen('F')) 
  {
    filament_detect.enable = false;
  }
}


inline void gcode_X78()
{
  SerialUSB.print("FSR0 ");
  SerialUSB.println(analogRead(0));
  SerialUSB.print("FSR1 ");
  SerialUSB.println(analogRead(1));
  SerialUSB.print("FSR2 ");
  SerialUSB.println(analogRead(2));
  SerialUSB.print("R_IO1 ");
  SerialUSB.println(analogRead(R_IO1));
  SerialUSB.print("R_IO2 ");
  SerialUSB.println(analogRead(R_IO2));
  SerialUSB.print("M_IO1 ");
  SerialUSB.println(digitalRead(M_IO1));
  SerialUSB.print("F0_STOP ");
  SerialUSB.println(digitalRead(F0_STOP));
  SerialUSB.print("F1_STOP ");
  SerialUSB.println(digitalRead(F1_STOP));
  SerialUSB.print("HOME_BTN_PIN ");
  SerialUSB.println(digitalRead(HOME_BTN_PIN));
  SerialUSB.print("U5FAULT ");
  SerialUSB.println(digitalRead(U5FAULT));

  if(code_seen('C')) {
    int val = code_value();
    if(val & 1) {
      analogWrite(U5EN, 255);
      SerialUSB.println("*U5EN ON");
    } else {
      analogWrite(U5EN, 0);
      SerialUSB.println("*U5EN OFF");
    }
  }

  //SerialUSB.println("ok");
}

inline void gcode_X111()
{
  SerialUSB.print("CRTL VERSION ");
  SerialUSB.print(FIRMWARE_VERSION);

  #ifdef VERSION_CONTROL
    SerialUSB.print(" ");
    SerialUSB.print(VERSION_CONTROL);
  #endif
  // TODO: become \r\n
  SerialUSB.println();
}

inline void gcode_X200()
{
	SerialUSB.print("Change LED status:");
	if (code_seen('C')) {
		int val = code_value();
		led_st.situational = val + 48;
		SerialUSB.println(led_st.situational);
	}
}

inline void gcode_X900()
{
    // X900 use to debug FSR
    float x = destination[X_AXIS];
    float y = destination[Y_AXIS];
    int counter = 5;

    if(code_seen('A')) {
        x = code_value();
    }
    if(code_seen('B')) {
        y = code_value();
    }
    if(code_seen('C')) {
        counter = code_value_short();
        if(counter > 200 || counter < 1) {
            SerialUSB.println("ER PARAM_ERROR C");
            return;
        }
    }

    // destination[X_AXIS] = x;
    // destination[Y_AXIS] = y;
    // destination[Z_AXIS] = 10;
    // prepare_move_raw();

    int min_value[3], max_value[3];
    float avg[3], sd[3];
    read_fsr_helper(counter, avg, sd, min_value, max_value);

    for(int i=0;i<3;i++) {
        SerialUSB.print("FSR ");
        SerialUSB.print(i);
        SerialUSB.print(" AVG ");
        SerialUSB.print(avg[i]);
        SerialUSB.print(" S ");
        SerialUSB.print(sd[i]);
        SerialUSB.print(" MIX ");
        SerialUSB.print(min_value[i]);
        SerialUSB.print(" MAX ");
        SerialUSB.println(max_value[i]);
    }
}


/*****************************************************
*** Process Commands and dispatch them to handlers ***
******************************************************/
bool process_commands() 
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

#ifdef DELTA

#if 1 //aven 0724 Mark as not used    
      case 29: // G29 Detailed Z-Probe, probes the bed at more points.
        gcode_G29();
        break;
      case 30:  // G30 Delta AutoCalibration
        gcode_G30();
        break;
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
      default:
        return false;
    }
  }

  else if(code_seen('M')) 
  {
    int gCode = code_value_short();  
    switch(gCode) 
    {
      case 17: //M17 - Enable/Power all stepper motors
        gcode_M17();
        break;
      case 31: //M31 take time since the start of the SD print or an M109 command
        gcode_M31();
        break;
      case 42: //M42 -Change pin status via gcode
        gcode_M42();
        break;
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

      case 140: // M140 Set bed temp
        gcode_M140();
        break;

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

      case 302: // No use
		  break;
      case 400: // M400 finish all moves
        gcode_M400();
        break;

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

#if defined(ENABLE_AUTO_BED_LEVELING) || defined(DELTA)
      case 666: //M666 Set Z probe offset or set delta endstop and geometry adjustment
        gcode_M666();
        break;
#endif //defined(ENABLE_AUTO_BED_LEVELING) || defined(DELTA)

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907();
        break;

      case 999: // M999: Restart after being Stopped
        gcode_M999();
        break;

#ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
      case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
        gcode_SET_Z_PROBE_OFFSET();
        break;
#endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
      default:
        return false;
    }
  }

  else if (code_seen('T')) 
  {
    gcode_T();
  }
  else if (code_seen('X')) 
  {
    int gCode = code_value_short();  
    switch(gCode) 
    {
      case 0:
        break;
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
        break;
      case 6:   
        gcode_X6();
        break;
      case 7:   
        gcode_X7();
        break;      
      case 8:   
        gcode_X8();
        break;
      case 78:
        gcode_X78();
        break;
      case 111:
        gcode_X111();
        break;
      case 200:
        gcode_X200();
        break;
      case 900:
        gcode_X900();
        break;
      default:
        return false;
    }
  }
  //aven_0415_2015 add cmd for S_LSA1 & S_LSA2 on/off end
  else if(code_seen('C')) {
    int cCode = code_value_short();

    switch(cCode) {
      case 1:
      // Enable/Disable lineno and sumcheck
        gcode_C1();
        break;
      case 2:
        gcode_C2();
        break;
      case 3:
        gcode_C3();
        break;
      case 4:
        gcode_C4();
        break;
      default:
        return false;
    }
  }
  else 
  {
    return false;
  }

  ClearToSend();
  return true;
}

void FlushSerialRequestResend() {
  //char cmdbuffer[bufindr][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(play_st.last_no + 1);
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


  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

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
  if(filament_detect.enable) {
    if((READ(F0_STOP)^FIL_RUNOUT_INVERTING) &&
       (millis() - filament_detect.last_trigger > 1000)) {

       SerialUSB.println("CTRL FILAMENTRUNOUT 0");
       filament_detect.last_trigger = millis();
    }
  }

  if (buflen < BUFSIZE - 1) get_command();

  unsigned long ms = millis();

  if (max_inactive_time && ms > previous_millis_cmd + max_inactive_time) kill();

  if (stepper_inactive_time && ms > previous_millis_cmd + stepper_inactive_time
      && !ignore_stepper_queue && !blocks_queued()) {
          if(play_st.enable_linecheck == 0) {
              // Only disable steppers while line check is disabled
              // because while line check is enabled it may printting something
              // and disable steppers may destory printting object.
              disable_all_steppers();
          }
      }

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
    Stopped_gcode_LastN = play_st.last_no; // Save last g_code for restart
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
