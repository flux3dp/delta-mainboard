
#define LED_OFF 0
#define LED_WAVE 1
#define LED_BLINK 2
#define LED_ON 3
#define LED_WAVE_2_ON 4
#define LED_WAVE_2_OFF 5
#define LED_STATIC 6

#define PI_NOT_DEFINED '?'
#define PI_WAKINGUP 'W'
#define PI_IDLE 'I'
#define PI_FATEL 'F'
#define PI_RUNNING 'R'
#define PI_RUNNING_WAIT_HEAD 'H'
#define PI_PAUSED 'P'
#define PI_ERROR 'E'
#define PI_UPDATE 'U'
#define PI_SLEEP 'S'
#define PI_STARTING_TASK 's'

#define PI_WIFI_CONNECTED 'C'
#define PI_WIFI_ASSOCOATING 'A'
#define PI_WIFI_DISCONNECTED 'D'

typedef struct LedStatus {
  char situational;
  char wifi;
  unsigned long last_update;
  char god_mode;

  char mode[3];
  float param_a[3];
  float param_b[3];
} LedStatus;

// manage_led
inline float _led_wave_atom(float a, float b) {
  float v = abs(1 - fmod(a * (millis() - b), 2));
  if(v < 0.15) return v * 1.04533;
  else if(v < 0.85) return v * 0.336143 + 0.106379;
  else return v * 4.05267 - 3.05267;
}
#define _led_blink_atom(a, b) abs(1 - fmod(a * (millis() - b), 2))
// #define _led_wave_atom(a, b) pow(sin(a * ((float)millis() - b)), 2)
#define _led_blink(i) _led_blink_atom(led_st.param_a[i], led_st.param_b[i])
#define _led_wave(i) _led_wave_atom(led_st.param_a[i], led_st.param_b[i])
