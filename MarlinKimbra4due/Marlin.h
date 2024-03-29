// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#ifdef __SAM3X8E__
  #include "HAL.h"
#else
  #include <util/delay.h>
  #include <avr/eeprom.h>
#endif

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "fastio.h"
#include "Configuration.h"

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define BIT(b) (1<<(b))
#define TEST(n,b) (((n)&BIT(b))!=0)
#define RADIANS(d) ((d)*M_PI/180.0)
#define DEGREES(r) ((d)*180.0/M_PI)

// Arduino < 1.0.0 does not define this, so we need to do it ourselves
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p) + 0xA0)
#endif

#ifdef AT90USB
  #include "HardwareSerial.h"
#endif

#ifndef __SAM3X8E__
  #include "MarlinSerial.h"
#endif

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include "WString.h"

#ifdef AT90USB
  #ifdef BTENABLED
    #define MYSERIAL bt
  #else
    #define MYSERIAL SerialUSB
  #endif // BTENABLED
#else
  #ifdef __SAM3X8E__
    #define MYSERIAL SerialUSB
  #else
    #define MYSERIAL MSerial
  #endif
#endif

#define SERIAL_CHAR(x) MYSERIAL.write(x)
#define SERIAL_EOL SERIAL_CHAR('\n')

#define SERIAL_PROTOCOLCHAR(x) SERIAL_CHAR(x)
#define SERIAL_PROTOCOL(x) MYSERIAL.print(x)
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y)
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x))
#define SERIAL_PROTOCOLLN(x) do{ MYSERIAL.print(x),MYSERIAL.write('\n'); }while(0)
#define SERIAL_PROTOCOLLNPGM(x) do{ serialprintPGM(PSTR(x)),MYSERIAL.write('\n'); }while(0)

extern const char errormagic[] PROGMEM;
extern const char echomagic[] PROGMEM;

#define SERIAL_ERROR_START serialprintPGM(errormagic)
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic)
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) do{ serial_echopair_P(PSTR(name),(value)); }while(0)

void serial_echopair_P(const char *s_P, float v);
void serial_echopair_P(const char *s_P, double v);
void serial_echopair_P(const char *s_P, unsigned long v);


// Things to write to serial from Program memory. Saves 400 to 2k of RAM.
FORCE_INLINE void serialprintPGM(const char *str) {
  char ch;
  while ((ch = pgm_read_byte(str))) {
    MYSERIAL.write(ch);
    str++;
  }
}

void get_command();
bool process_commands();

void manage_inactivity(bool ignore_stepper_queue=false);

#if defined(DUAL_X_CARRIAGE) && HAS_X_ENABLE && HAS_X2_ENABLE
  #define  enable_x() do { X_ENABLE_WRITE( X_ENABLE_ON); X2_ENABLE_WRITE( X_ENABLE_ON); } while (0)
  #define disable_x() do { X_ENABLE_WRITE(!X_ENABLE_ON); X2_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; } while (0)
#elif HAS_X_ENABLE
  #define  enable_x() X_ENABLE_WRITE( X_ENABLE_ON)
  #define disable_x() { X_ENABLE_WRITE(!X_ENABLE_ON); axis_known_position[X_AXIS] = false; }
#else
  #define enable_x() ;
  #define disable_x() ;
#endif

#if HAS_Y_ENABLE
  #ifdef Y_DUAL_STEPPER_DRIVERS
    #define  enable_y() { Y_ENABLE_WRITE( Y_ENABLE_ON); Y2_ENABLE_WRITE(Y_ENABLE_ON); }
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); Y2_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #else
    #define  enable_y() Y_ENABLE_WRITE( Y_ENABLE_ON)
    #define disable_y() { Y_ENABLE_WRITE(!Y_ENABLE_ON); axis_known_position[Y_AXIS] = false; }
  #endif
#else
  #define enable_y() ;
  #define disable_y() ;
#endif

#if HAS_Z_ENABLE
  #ifdef Z_DUAL_STEPPER_DRIVERS
    #define  enable_z() { Z_ENABLE_WRITE( Z_ENABLE_ON); Z2_ENABLE_WRITE(Z_ENABLE_ON); }
    #define disable_z() { Z_ENABLE_WRITE(!Z_ENABLE_ON); Z2_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #else
    #define  enable_z() Z_ENABLE_WRITE( Z_ENABLE_ON)
    #define disable_z() { Z_ENABLE_WRITE(!Z_ENABLE_ON); axis_known_position[Z_AXIS] = false; }
  #endif
#else
  #define enable_z() ;
  #define disable_z() ;
#endif

#if HAS_E0_ENABLE
  #define enable_e0()  E0_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e0() E0_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e0()  /* nothing */
  #define disable_e0() /* nothing */
#endif

#if (DRIVER_EXTRUDERS > 1) && HAS_E1_ENABLE
  #define enable_e1()  E1_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e1() E1_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e1()  /* nothing */
  #define disable_e1() /* nothing */
#endif

#if (DRIVER_EXTRUDERS > 2) && HAS_E2_ENABLE
  #define enable_e2()  E2_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e2() E2_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e2()  /* nothing */
  #define disable_e2() /* nothing */
#endif

#if (DRIVER_EXTRUDERS > 3) && HAS_E3_ENABLE
  #define enable_e3()  E3_ENABLE_WRITE( E_ENABLE_ON)
  #define disable_e3() E3_ENABLE_WRITE(!E_ENABLE_ON)
#else
  #define enable_e3()  /* nothing */
  #define disable_e3() /* nothing */
#endif

#define disable_e() {disable_e0(); disable_e1(); disable_e2(); disable_e3();}

/**
 * The axis order in all axis related arrays is X, Y, Z, E
 */
#define NUM_AXIS 4

/**
 * Axis indices as enumerated constants
 *
 * A_AXIS and B_AXIS are used by COREXY printers
 * X_HEAD and Y_HEAD is used for systems that don't have a 1:1 relationship between X_AXIS and X Head movement, like CoreXY bots.
 */
enum AxisEnum {X_AXIS=0, Y_AXIS=1, A_AXIS=0, B_AXIS=1, Z_AXIS=2, E_AXIS=3, X_HEAD=4, Y_HEAD=5};

void enable_all_steppers();
void disable_all_steppers();

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void manage_led(void);
void manage_led_plus(void);
#ifdef DELTA
float probe_bed(float x, float y);
void set_delta_constants();
void home_delta_axis();
void calibration_report();
void bed_probe_all();
void set_default_z_probe_offset();
void set_delta_constants();
void save_carriage_positions(int position_num);
void calculate_delta(float cartesian[3]);
void adjust_delta(float cartesian[3]);
void prepare_move_raw();
extern float delta[3];
extern float delta_tmp[3];
extern float delta_tower1_x,delta_tower1_y;
extern float delta_tower2_x,delta_tower2_y;
extern float delta_tower3_x,delta_tower3_y;
#endif
#ifdef SCARA
  void calculate_delta(float cartesian[3]);
  void calculate_SCARA_forward_Transform(float f_scara[3]);
#endif
void prepare_move();
void kill();
void Stop();

#ifdef FILAMENT_RUNOUT_SENSOR
  void filrunout();
#endif

bool IsStopped();

bool enquecommand(const char *cmd); //put a single ASCII command at the end of the current buffer or return false when it is full
void enquecommands_P(const char *cmd); //put one or many ASCII commands at the end of the current buffer, read from flash

void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

extern unsigned long previous_millis_cmd;
inline void refresh_cmd_timeout() { previous_millis_cmd = millis(); }

#ifdef FAST_PWM_FAN
  void setPwmFrequency(uint8_t pin, int val);
#endif

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern bool volumetric_enabled;
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float filament_size[EXTRUDERS]; // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
extern float volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS];
extern float destination[NUM_AXIS];
extern float home_offset[3];

// Hotend offset
#if HOTENDS > 1
  #ifndef DUAL_X_CARRIAGE
    #define NUM_HOTEND_OFFSETS 2 // only in XY plane
  #else
    #define NUM_HOTEND_OFFSETS 3 // supports offsets in XYZ plane
  #endif
  extern float hotend_offset[NUM_HOTEND_OFFSETS][HOTENDS];
#endif // HOTENDS > 1

#ifdef NPR2
  extern int old_color; // old color for system NPR2
#endif

#ifdef DELTA
  extern float z_probe_offset[3];
  extern float endstop_adj[3];
  extern float tower_adj[6];
  extern float delta_radius;
  extern float delta_diagonal_rod;
#elif defined(Z_DUAL_ENDSTOPS)
extern float z_endstop_adj;
#endif

#ifdef SCARA
  extern float axis_scaling[3];  // Build size scaling
#endif

extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern float lastpos[4];
extern float zprobe_zoffset;
extern int fanSpeed;

#ifdef BARICUDA
  extern int ValvePressure;
  extern int EtoPPressure;
#endif

#ifdef FAN_SOFT_PWM
  extern unsigned char fanSpeedSoftPwm;
#endif

#if HAS_FILAMENT_SENSOR
  extern float filament_width_nominal;    //holds the theoretical filament diameter ie., 3.00 or 1.75
  extern bool filament_sensor;            //indicates that filament sensor readings should control extrusion
  extern float filament_width_meas;       //holds the filament diameter as accurately measured
  extern signed char measurement_delay[]; //ring buffer to delay measurement
  extern int delay_index1, delay_index2;  //index into ring buffer
  extern float delay_dist;                //delay distance counter
  extern int meas_delay_cm;               //delay distance
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  extern float power_consumption_meas;          //holds the power consumption as accurately measured
  extern unsigned long power_consumption_hour;  //holds the power consumption per hour as accurately measured
  extern unsigned long startpower;
  extern unsigned long stoppower;
#endif

#ifdef IDLE_OOZING_PREVENT
  extern bool idleoozing_enabled;
#endif

#ifdef FWRETRACT
  extern bool autoretract_enabled;
  extern bool retracted[EXTRUDERS];
  extern float retract_length, retract_length_swap, retract_feedrate, retract_zlift;
  extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate;
#endif

#ifdef EASY_LOAD
  extern bool allow_lengthy_extrude_once; // for load/unload
#endif

#ifdef LASERBEAM
  extern int laser_ttl_modulation;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

// Handling multiple extruders pins
extern uint8_t active_extruder;
extern uint8_t active_driver;

#ifdef DIGIPOT_I2C
  extern void digipot_i2c_set_current( int channel, float current );
  extern void digipot_i2c_init();
#endif

// Debug with repetier
extern uint8_t debugLevel;
extern inline bool debugDryrun() {
  return ((debugLevel & 8) != 0);
}

#ifdef FIRMWARE_TEST
  void FirmwareTest();
#endif

extern void calculate_volumetric_multipliers();

#define FIREWARE_VERSION "1.0.0"

typedef struct FilamentDetect {
  bool enable;
  unsigned long last_trigger;
} FilamentDetect;

//#define NO_METAL_PLATE_FSR_VALUE 4095

typedef struct GlobalVariable {
  unsigned int home_btn_press;
};

typedef struct PlayStatus {
  int enable_linecheck;
  int stashed;
  unsigned long last_no;
  int stashed_laser_pwm;
  float stashed_position[NUM_AXIS];
  float stashed_feedrate;
  uint8_t stashed_extruder;
  uint8_t stashed_mode;
} PlayStatus;

enum STASH_MODE { STASH_PEN  = 0, STASH_LASER = 1, STASH_PRINT = 2, STASH_LOADED_FILAMENT = 3};

//To detect G28 that doesn't touch micro switch
extern volatile bool endstop_x_hit;
extern volatile bool endstop_y_hit;
extern volatile bool endstop_z_hit;
static volatile bool endstop_has_hit[3];
#endif //MARLIN_H

//Eliminating backlash
extern float backlash_offset[4];

/*
* PWM Capture using Arduino Due Timer Counters
*
* Available channels:
*   TC    Chan   NVIC irq   Handler       PMC ID   Arduino Pin		IO
*   TC0   0      TC0_IRQn   TC0_Handler   ID_TC0   D2     (TIOA0)	PB25
*   TC0   1      TC1_IRQn   TC1_Handler   ID_TC1   D61/A7 (TIOA1)	PA2
*   TC2   0      TC6_IRQn   TC6_Handler   ID_TC6   D5     (TIOA6)	PC25
*   TC2   1      TC7_IRQn   TC7_Handler   ID_TC7   D3     (TIOA7)	PC28
*   TC2   2      TC8_IRQn   TC8_Handler   ID_TC8   D11    (TIOA8)	PD7
*
* Change the following defines to use different channels as input.
*
*/
#define PWM_OUT_PIN 13
#define CAPTURE_TC TC0//TC2//TC2//TC0//TC2//
#define CAPTURE_CHANNEL 0//2//0//1//1//
#define CAPTURE_IRQn TC0_IRQn//TC8_IRQn//TC6_IRQn//TC1_IRQn//TC7_IRQn//
#define CAPTURE_Handler TC0_Handler//TC8_Handler//TC6_Handler//TC1_Handler//TC7_Handler//
#define CAPTURE_ID ID_TC0//ID_TC8//ID_TC6//ID_TC1//ID_TC7//
#define CAPTURE_PIN 2//11//5//A7//3//
#define CAPTURE_CLOCK_SELECTION TC_CMR_TCCLKS_TIMER_CLOCK3
#define ComPort SerialUSB
// clock divisors corresponding to CAPTURE_CLOCK_SELECTION

/*
Hardware version
default version:Flux Delta
VERSION_0_PIN   VERSION_1_PIN   VERSION_2_PIN   HARDWARE
    HIGH            HIGH            HIGH        Flux Delta
    LOW             HIGH            HIGH        Flux Delta plus
*/
#define FLUX_DELTA      0
#define FLUX_DELTA_PLUS 1 
#define FLUX_DELTA_UPGRADE 2

