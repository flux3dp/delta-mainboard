/**
 * Catalan
 *
 * LCD Menu Messages
 * See also documentation/LCDLanguageFont.md
 *
 */
#ifndef LANGUAGE_CA_H
#define LANGUAGE_CA_H

//#define MAPPER_NON 
#define MAPPER_C2C3  // because of "ó"
// Define SIMULATE_ROMFONT to see what is seen on the character based display defined in Configuration.h
//#define SIMULATE_ROMFONT
#define DISPLAY_CHARSET_ISO10646_1

#define WELCOME_MSG                         MACHINE_NAME " preparada."
#define MSG_SD_INSERTED                     "SD detectada."
#define MSG_SD_REMOVED                      "SD expulsada."
#define MSG_MAIN                            "Menu principal"
#define MSG_AUTOSTART                       "Inici automatic"
#define MSG_DISABLE_STEPPERS                "Apagar motors"
#define MSG_AUTO_HOME                       "Home global"
#define MSG_BED_SETTING                     "Bed Setting"
#define MSG_LP_INTRO                        " Leveling bed...       Press to start  "
#define MSG_LP_1                            " Adjust first point  & Press the button"
#define MSG_LP_2                            " Adjust second point & Press the button"
#define MSG_LP_3                            " Adjust third point  & Press the button"
#define MSG_LP_4                            " Adjust fourth point & Press the button"
#define MSG_LP_5                            "     Is it ok?         Press to end"       
#define MSG_LP_6                            " BED leveled!"
#define MSG_SET_HOME_OFFSETS                "Set home offsets"
#define MSG_SET_ORIGIN                      "Establir origen"
#define MSG_PREHEAT_PLA                     "Preescalfar PLA"
#define MSG_PREHEAT_PLA_ALL                 "Preesc. tot PLA All"
#define MSG_PREHEAT_PLA_BEDONLY             "Preesc. llit PLA"
#define MSG_PREHEAT_PLA_SETTINGS            "Configuració PLA"
#define MSG_PREHEAT_ABS                     "Preescalfar ABS"
#define MSG_PREHEAT_ABS_ALL                 "Preheat ABS All"
#define MSG_PREHEAT_ABS_BEDONLY             "Preesc. llit ABS"
#define MSG_PREHEAT_ABS_SETTINGS            "Configuració ABS"
#define MSG_PREHEAT_GUM                     "Preheat GUM"
#define MSG_PREHEAT_GUM_ALL                 "Preheat GUM All"
#define MSG_PREHEAT_GUM_BEDONLY             "Preheat GUM Bed"
#define MSG_PREHEAT_GUM_SETTINGS            "Preheat GUM conf"
#define MSG_COOLDOWN                        "Refredar"
#define MSG_SWITCH_PS_ON                    "Switch power on"
#define MSG_SWITCH_PS_OFF                   "Switch power off"
#define MSG_EXTRUDE                         "Extruir"
#define MSG_RETRACT                         "Refredar"
#define MSG_MOVE_AXIS                       "Moure eixos"
#define MSG_MOVE_X                          "Moure X"
#define MSG_MOVE_Y                          "Moure Y"
#define MSG_MOVE_Z                          "Moure Z"
#define MSG_MOVE_E                          "Extrusor"
#define MSG_MOVE_01MM                       "Moure 0.1mm"
#define MSG_MOVE_1MM                        "Moure 1mm"
#define MSG_MOVE_10MM                       "Moure 10mm"
#define MSG_SPEED                           "Velocitat"
#define MSG_NOZZLE                          "Nozzle"
#define MSG_BED                             "Llit"
#define MSG_FAN_SPEED                       "Vel. Ventilador"
#define MSG_FLOW                            "Fluxe"
#define MSG_CONTROL                         "Control"
#define MSG_MIN                             " \002 Min"
#define MSG_MAX                             " \002 Max"
#define MSG_FACTOR                          " \002 Fact"
#define MSG_IDLEOOZING                      "Anti oozing"
#define MSG_AUTOTEMP                        "Autotemp"
#define MSG_ON                              "On "
#define MSG_OFF                             "Off"
#define MSG_PID_P                           "PID-P"
#define MSG_PID_I                           "PID-I"
#define MSG_PID_D                           "PID-D"
#define MSG_ACC                             "Accel"
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
#define MSG_XSTEPS                          "X passos/mm"
#define MSG_YSTEPS                          "Y passos/mm"
#define MSG_ZSTEPS                          "Z passos/mm"
#define MSG_E0STEPS                         "E0 passos/mm"
#define MSG_E1STEPS                         "E1 passos/mm"
#define MSG_E2STEPS                         "E2 passos/mm"
#define MSG_E3STEPS                         "E3 passos/mm"
#define MSG_TEMPERATURE                     "Temperatura"
#define MSG_MOTION                          "Moviment"
#define MSG_VOLUMETRIC                      "Filament"
#define MSG_VOLUMETRIC_ENABLED              "E in mm3"
#define MSG_FILAMENT_SIZE_EXTRUDER          "Fil. Dia."
#define MSG_CONTRAST                        "Contrast de LCD"
#define MSG_STORE_EPROM                     "Desar a memoria"
#define MSG_LOAD_EPROM                      "Carregar de mem."
#define MSG_RESTORE_FAILSAFE                "Rest. emergencia"
#define MSG_REFRESH                         "Refrescar"
#define MSG_WATCH                           "Pantalla Info."
#define MSG_PREPARE                         "Preparar"
#define MSG_TUNE                            "Calibrar"
#define MSG_PAUSE_PRINT                     "Pausa imp."
#define MSG_RESUME_PRINT                    "Reprendre imp."
#define MSG_STOP_PRINT                      "Parar inp."
#define MSG_CARD_MENU                       "Imprimir de SD"
#define MSG_NO_CARD                         "-Sense targeta SD"
#define MSG_DWELL                           "Repos..."
#define MSG_USERWAIT                        "Esperant usuari.."
#define MSG_RESUMING                        "Reprenent imp."
#define MSG_PRINT_ABORTED                   "Print aborted"
#define MSG_NO_MOVE                         "Sense moviment."
#define MSG_KILLED                          "PARADA DE EMERG. "
#define MSG_STOPPED                         "ATURAT. "
#define MSG_CONTROL_RETRACT                 "Retreure mm"
#define MSG_CONTROL_RETRACT_SWAP            "Swap Retreure mm"
#define MSG_CONTROL_RETRACTF                "Retreure  F"
#define MSG_CONTROL_RETRACT_ZLIFT           "Aixecar mm"
#define MSG_CONTROL_RETRACT_RECOVER         "DesRet +mm"
#define MSG_CONTROL_RETRACT_RECOVER_SWAP    "Swap DesRet +mm"
#define MSG_CONTROL_RETRACT_RECOVERF        "DesRet  F"
#define MSG_AUTORETRACT                     "AutoRetr."
#define MSG_FILAMENTCHANGE                  "Canviar filament"
#define MSG_INIT_SDCARD                     "Iniciant SD"
#define MSG_CNG_SDCARD                      "Canviar SD"
#define MSG_ZPROBE_OUT                      "Z probe out. bed"
#define MSG_POSITION_UNKNOWN                "Home X/Y abans Z"
#define MSG_ZPROBE_ZOFFSET                  "Z Offset"
#define MSG_BABYSTEP_X                      "Babystep X"
#define MSG_BABYSTEP_Y                      "Babystep Y"
#define MSG_BABYSTEP_Z                      "Babystep Z"
#define MSG_ENDSTOP_ABORT                   "Endstop abort"

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

#endif // LANGUAGE_EN_H
