/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h 

#include "planner.h"
#include "stepper_indirection.h"

#if MB(ALLIGATOR)
  #include "ExternalDac.h"
#endif

#if DRIVER_EXTRUDERS > 3
  #define E_STEP_WRITE(v) { if(current_block->active_driver == 3) { E3_STEP_WRITE(v); } else { if(current_block->active_driver == 2) { E2_STEP_WRITE(v); } else { if(current_block->active_driver == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}}}
  #define NORM_E_DIR() { if(current_block->active_driver == 3) { E3_DIR_WRITE( !INVERT_E3_DIR); } else { if(current_block->active_driver == 2) { E2_DIR_WRITE(!INVERT_E2_DIR); } else { if(current_block->active_driver == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}}}
  #define REV_E_DIR() { if(current_block->active_driver == 3) { E3_DIR_WRITE(INVERT_E3_DIR); } else { if(current_block->active_driver == 2) { E2_DIR_WRITE(INVERT_E2_DIR); } else { if(current_block->active_driver == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}}}
#elif DRIVER_EXTRUDERS > 2
  #define E_STEP_WRITE(v) { if(current_block->active_driver == 2) { E2_STEP_WRITE(v); } else { if(current_block->active_driver == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}}
  #define NORM_E_DIR() { if(current_block->active_driver == 2) { E2_DIR_WRITE(!INVERT_E2_DIR); } else { if(current_block->active_driver == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}}
  #define REV_E_DIR() { if(current_block->active_driver == 2) { E2_DIR_WRITE(INVERT_E2_DIR); } else { if(current_block->active_driver == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}}
#elif DRIVER_EXTRUDERS > 1
  #ifndef DUAL_X_CARRIAGE
    #define E_STEP_WRITE(v) { if(current_block->active_driver == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}
    #define NORM_E_DIR() { if(current_block->active_driver == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}
    #define REV_E_DIR() { if(current_block->active_driver == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}
  #else
    extern bool extruder_duplication_enabled;
    #define E_STEP_WRITE(v) { if(extruder_duplication_enabled) { E0_STEP_WRITE(v); E1_STEP_WRITE(v); } else if(current_block->active_driver == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}
    #define NORM_E_DIR() { if(extruder_duplication_enabled) { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); } else if(current_block->active_driver == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}
    #define REV_E_DIR() { if(extruder_duplication_enabled) { E0_DIR_WRITE(INVERT_E0_DIR); E1_DIR_WRITE(INVERT_E1_DIR); } else if(current_block->active_driver == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}
  #endif  
#else
  #define E_STEP_WRITE(v) E0_STEP_WRITE(v)
  #define NORM_E_DIR() E0_DIR_WRITE(!INVERT_E0_DIR)
  #define REV_E_DIR() E0_DIR_WRITE(INVERT_E0_DIR)
#endif //DRIVER_EXTRUDERS

#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
extern bool abort_on_endstop_hit;
#endif

// Initialize and start the stepper motor subsystem
void st_init();

// Block until all buffered steps are executed
void st_synchronize();

// Set current position in steps
void st_set_position(const long &x, const long &y, const long &z, const long &e);
void st_set_e_position(const long &e);

// Get current position in steps
long st_get_position(uint8_t axis);

#ifdef ENABLE_AUTO_BED_LEVELING
// Get current position in mm
float st_get_position_mm(uint8_t axis);
#endif  //ENABLE_AUTO_BED_LEVELING

// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();

  
void checkHitEndstops(); //call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered
void endstops_hit_on_purpose(); //avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops();

void enable_endstops(bool check); // Enable/disable endstop checking

void checkStepperErrors(); //Print errors detected by the stepper

void finishAndDisableSteppers();

extern block_t *current_block;  // A pointer to the block currently being traced

void quickStop();

void digitalPotWrite(int address, int value);
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
void microstep_mode(uint8_t driver, uint8_t stepping);
void digipot_init();
void digipot_current(uint8_t driver, int current);
void microstep_init();
void microstep_readings();

#ifdef Z_DUAL_ENDSTOPS
  void In_Homing_Process(bool state);
  void Lock_z_motor(bool state);
  void Lock_z2_motor(bool state);
#endif

#ifdef BABYSTEPPING
  void babystep(const uint8_t axis,const bool direction); // perform a short step with a single stepper motor, outside of any convention
#endif //BABYSTEPPING

#ifdef NPR2 //Multiextruder
  void colorstep(long csteps,const bool direction);
#endif //NPR2

#endif //stepper_h
