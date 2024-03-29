/**
 * Dutch
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_NL_H
#define LANGUAGE_NL_H

#define MAPPER_NON
// Define SIMULATE_ROMFONT to see what is seen on the character based display defined in Configuration.h
//#define SIMULATE_ROMFONT
#define DISPLAY_CHARSET_ISO10646_1

#define WELCOME_MSG                         MACHINE_NAME " gereed."
#define MSG_SD_INSERTED                     "Kaart ingestoken"
#define MSG_SD_REMOVED                      "Kaart verwijderd"
#define MSG_MAIN                            "Main"
#define MSG_AUTOSTART                       "Autostart"
#define MSG_DISABLE_STEPPERS                "Motoren uit"
#define MSG_AUTO_HOME                       "Auto home"
#define MSG_BED_SETTING                     "Bed Setting"
#define MSG_LP_INTRO                        " Leveling bed...       Press to start  "
#define MSG_LP_1                            " Adjust first point  & Press the button"
#define MSG_LP_2                            " Adjust second point & Press the button"
#define MSG_LP_3                            " Adjust third point  & Press the button"
#define MSG_LP_4                            " Adjust fourth point & Press the button"
#define MSG_LP_5                            "     Is it ok?         Press to end"       
#define MSG_LP_6                            " BED leveled!"
#define MSG_SET_HOME_OFFSETS                "Set home offsets"
#define MSG_SET_ORIGIN                      "Nulpunt instellen"
#define MSG_PREHEAT_PLA                     "PLA voorverwarmen"
#define MSG_PREHEAT_PLA_ALL                 "PLA voorverw. aan"
#define MSG_PREHEAT_PLA_BEDONLY             "PLA voorverw. Bed"
#define MSG_PREHEAT_PLA_SETTINGS            "PLA verw. conf"
#define MSG_PREHEAT_ABS                     "ABS voorverwarmen"
#define MSG_PREHEAT_ABS_ALL                 "ABS voorverw. aan"
#define MSG_PREHEAT_ABS_BEDONLY             "ABS voorverw. Bed"
#define MSG_PREHEAT_ABS_SETTINGS            "ABS verw. conf"
#define MSG_PREHEAT_GUM                     "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 "Preheat GUM All"
#define MSG_PREHEAT_GUM_BEDONLY             "Preheat GUM Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "Preheat GUM conf"
#define MSG_COOLDOWN                        "Afkoelen"
#define MSG_SWITCH_PS_ON                    "Stroom aan"
#define MSG_SWITCH_PS_OFF                   "Stroom uit"
#define MSG_EXTRUDE                         "Extrude"
#define MSG_RETRACT                         "Retract"
#define MSG_MOVE_AXIS                       "As verplaatsen"
#define MSG_MOVE_X                          "Verplaats X"
#define MSG_MOVE_Y                          "Verplaats Y"
#define MSG_MOVE_Z                          "Verplaats Z"
#define MSG_MOVE_E                          "Extruder"
#define MSG_MOVE_01MM                       "Verplaats 0.1mm"
#define MSG_MOVE_1MM                        "Verplaats 1mm"
#define MSG_MOVE_10MM                       "Verplaats 10mm"
#define MSG_SPEED                           "Snelheid"
#define MSG_NOZZLE                          "Nozzle"
#define MSG_BED                             "Bed"
#define MSG_FAN_SPEED                       "Fan snelheid"
#define MSG_FLOW                            "Flow"
#define MSG_CONTROL                         "Control"
#define MSG_MIN                             " " STR_THERMOMETER " Min"
#define MSG_MAX                             " " STR_THERMOMETER " Max"
#define MSG_FACTOR                          " " STR_THERMOMETER " Fact"
#define MSG_AUTOTEMP                        "Autotemp"
#define MSG_ON                              "Aan "
#define MSG_OFF                             "Uit"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_ACC                             "Versn"
#define MSG_VXY_JERK                        "Vxy-jerk"
#define MSG_VZ_JERK                         "Vz-jerk"
#define MSG_VE_JERK                         "Ve-jerk"
#define MSG_VMAX                            "Vmax "
#define MSG_X                               "x"
#define MSG_Y                               "y"
#define MSG_Z                               "z"
#define MSG_E                               "e"
#define MSG_VMIN                            "Vmin"
#define MSG_VTRAV_MIN                       "VTrav min"
#define MSG_AMAX                            "Amax "
#define MSG_A_RETRACT                       "A-retract"
#define MSG_A_TRAVEL                        "A-travel"
#define MSG_XSTEPS                          "X steps/mm"
#define MSG_YSTEPS                          "Y steps/mm"
#define MSG_ZSTEPS                          "Z steps/mm"
#define MSG_E0STEPS                         "E0 steps/mm"
#define MSG_E1STEPS                         "E1 steps/mm"
#define MSG_E2STEPS                         "E2 steps/mm"
#define MSG_E3STEPS                         "E3 steps/mm"
#define MSG_TEMPERATURE                     "Temperatuur"
#define MSG_MOTION                          "Beweging"
#define MSG_VOLUMETRIC                      "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm" STR_h3
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "LCD contrast"
#define MSG_STORE_EPROM                     "Geheugen opslaan"
#define MSG_LOAD_EPROM                      "Geheugen laden"
#define MSG_RESTORE_FAILSAFE                "Noodstop reset"
#define MSG_REFRESH                         "Ververs"
#define MSG_WATCH                           "Info scherm"
#define MSG_PREPARE                         "Voorbereiden"
#define MSG_TUNE                            "Afstellen"
#define MSG_PAUSE_PRINT                     "Print pauzeren"
#define MSG_RESUME_PRINT                    "Print hervatten"
#define MSG_STOP_PRINT                      "Print stoppen"
#define MSG_CARD_MENU                       "Print van SD"
#define MSG_NO_CARD                         "Geen SD kaart"
#define MSG_DWELL                           "Slapen..."
#define MSG_USERWAIT                        "Wachten..."
#define MSG_RESUMING                        "Print hervatten"
#define MSG_PRINT_ABORTED                   "Print aborted"
#define MSG_NO_MOVE                         "Geen beweging."
#define MSG_KILLED                          "AFGEBROKEN. "
#define MSG_STOPPED                         "GESTOPT. "
#define MSG_CONTROL_RETRACT                 "Retract mm"
#define MSG_CONTROL_RETRACT_SWAP            "Ruil Retract mm"
#define MSG_CONTROL_RETRACTF                "Retract  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Hop mm"
#define MSG_CONTROL_RETRACT_RECOVER         "UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Ruil UnRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "UnRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENTCHANGE                  "Verv. Filament"
#define MSG_INIT_SDCARD                     "Init. SD kaart"
#define MSG_CNG_SDCARD                      "Verv. SD card"
#define MSG_ZPROBE_OUT                      "Z probe uit. bed"
#define MSG_POSITION_UNKNOWN                "Home X/Y voor Z"
#define MSG_ZPROBE_ZOFFSET                  "Z Offset"
#define MSG_BABYSTEP_X                      "Babystap X"
#define MSG_BABYSTEP_Y                      "Babystap Y"
#define MSG_BABYSTEP_Z                      "Babystap Z"
#define MSG_ENDSTOP_ABORT                   "Endstop afbr."

#define MSG_HEATING_FAILED_LCD              "Heating failed"
#define MSG_ERR_REDUNDANT_TEMP              "Err: REDUNDANT TEMP ERROR"
#define MSG_THERMAL_RUNAWAY                 "THERMAL RUNAWAY"
#define MSG_ERR_MAXTEMP                     "Err: MAXTEMP"
#define MSG_ERR_MINTEMP                     "Err: MINTEMP"
#define MSG_ERR_MAXTEMP_BED                 "Err: MAXTEMP BED"

#ifdef DELTA
  #define MSG_DELTA_CALIBRATE               "Delta Calibration"
  #define MSG_DELTA_CALIBRATE_X             "Calibrate X"
  #define MSG_DELTA_CALIBRATE_Y             "Calibrate Y"
  #define MSG_DELTA_CALIBRATE_Z             "Calibrate Z"
  #define MSG_DELTA_CALIBRATE_CENTER        "Calibrate Center"
#endif // DELTA

#define MSG_LASER                           "Laser Preset"
#define MSG_CONFIG                          "Configuration"
#define MSG_BAUDRATE                        "Baudrate"
#define MSG_E_BOWDEN_LENGTH                 "Extrude " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_R_BOWDEN_LENGTH                 "Retract " STRINGIFY(BOWDEN_LENGTH) "mm"
#define MSG_PURGE_XMM                       "Purge " STRINGIFY(LCD_PURGE_LENGTH) "mm"
#define MSG_RETRACT_XMM                     "Retract " STRINGIFY(LCD_RETRACT_LENGTH) "mm"

#ifdef FIRMWARE_TEST
  #define MSG_FWTEST_YES                    "Put the Y command to go next"
  #define MSG_FWTEST_NO                     "Put the N command to go next"
  #define MSG_FWTEST_YES_NO                 "Put the Y or N command to go next"
  #define MSG_FWTEST_ENDSTOP_ERR            "ENDSTOP ERROR! Check wire and connection"
  #define MSG_FWTEST_PRESS                  "Press and hold the endstop "
  #define MSG_FWTEST_INVERT                 "Reverse value in "
  #define MSG_FWTEST_XAXIS                  "Has the nozzle moved to the right?"
  #define MSG_FWTEST_YAXIS                  "Has the nozzle moved forward?"
  #define MSG_FWTEST_ZAXIS                  "Has the nozzle moved up?"
  #define MSG_FWTEST_01                     "Manually move the axes X, Y and Z away from the endstop"
  #define MSG_FWTEST_02                     "Do you want check ENDSTOP?"
  #define MSG_FWTEST_03                     "Start check ENDSTOP"
  #define MSG_FWTEST_04                     "Start check MOTOR"
  #define MSG_FWTEST_ATTENTION              "ATTENTION! Check that the three axes are more than 5 mm from the endstop!"
  #define MSG_FWTEST_END                    "Finish Test. Disable FIRMWARE_TEST and recompile."
#endif // FIRMWARE_TEST

#endif // LANGUAGE_NL_H
