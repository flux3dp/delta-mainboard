
#define LED_OFF 0
#define LED_WAVE 1
#define LED_BLINK 2
#define LED_ON 3
#define LED_WAVE_2_ON 4
#define LED_WAVE_2_OFF 5
#define LED_STATIC 6
#define LED_FLASH 7
#define LED_WAVE2 8

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
#define PI_WIFI_HOSTED 'h'
#define PI_STARTING_TASK 's'

#define PI_ERROR_1 '1'
#define PI_ERROR_2 '2'
#define PI_ERROR_3 '3'
#define PI_ERROR_4 '4'
#define PI_ERROR_5 '5'
#define PI_ERROR_6 '6'
#define PI_ERROR_7 '7'
#define PI_ERROR_8 '8'
#define PI_ERROR_9 '9'


#define PI_WIFI_CONNECTED 'C'
#define PI_WIFI_ASSOCOATING 'A'
#define PI_WIFI_DISCONNECTED 'D'
#define NoAction 'N'
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
// millis:49.71 days
inline float _led_wave_atom(float a, uint32_t b) {
    float n = a * (millis() - b);
    uint32_t x = (uint32_t)abs(n);
    float f = n - (float)x;
    float v = abs(1 - ((x%2)+f));
    //float v = abs(1 - fmod(a * (millis() - b), 2));
    if(v < 0.15) return v * 1.04533;
    else if(v < 0.85) return v * 0.336143 + 0.106379;
    else return v * 4.05267 - 3.05267;
}

inline int _led_special(int param_a, uint32_t param_b) {
    int delay_ratio = 1000 + param_a * 25;
    int led_cycle = param_a * 600 + delay_ratio;
    int offset = (millis() - param_b) % led_cycle;
    if((led_cycle - offset) < delay_ratio) {
    return 255;
    } else {
    return (((offset / 300) + 1) % 2) * 255;
    }
}

inline float _led_blink_atom(float a, uint32_t b) {
    float n = a * (millis() - b);
    uint32_t x = (uint32_t)abs(n);
    float f = n - (float)x;
    float v = abs(1 - ((x % 2) + f));
    //float v = abs(1 - fmod(a * (millis() - b), 2));
    return v;
}

//#define _led_blink_atom(a, b) abs(1 - fmod(a * (millis() - b), 2))
// #define _led_wave_atom(a, b) pow(sin(a * ((float)millis() - b)), 2)
#define _led_blink(i) _led_blink_atom(led_st.param_a[i], led_st.param_b[i])
#define _led_wave(i) _led_wave_atom(led_st.param_a[i], led_st.param_b[i])
