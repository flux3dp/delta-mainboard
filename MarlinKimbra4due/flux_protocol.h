
#include "Marlin.h"

extern struct PlayStatus play_st;


void inline report_ln(int buflen) {
    SERIAL_PROTOCOL("LN ");
    SERIAL_PROTOCOL(play_st.last_no);
    SERIAL_PROTOCOL(" ");
    SERIAL_PROTOCOL(buflen);
    SERIAL_PROTOCOL("\n");
}

void inline log_zprobe(float x, float y, float probe_value) {
    SERIAL_PROTOCOL("Bed Z-Height at X:");
    SERIAL_PROTOCOL(x);
    SERIAL_PROTOCOL(" Y:");
    SERIAL_PROTOCOL(y);
    SERIAL_PROTOCOL(" = ");
    SERIAL_PROTOCOL_F(probe_value, 4);
    SERIAL_PROTOCOL("\n");

    SERIAL_PROTOCOL("DATA ZPROBE ");
    SERIAL_PROTOCOL_F(probe_value, 4);
    SERIAL_PROTOCOL("\n");
}

void inline log_error_fsr(uint flag) {
    // flag:
    //    1: X-,  2: XO
    //    4: Y-,  8: YO
    //   16: Z-, 32: ZO

    int i;
    SERIAL_PROTOCOL("ER FSR");
    for(i=0;i<3;i++) {
        if(flag & (1 << (i * 2))) {
            SERIAL_PROTOCOL(" ");
            SERIAL_PROTOCOL((char)(88 + i)); // char 88 = X
            SERIAL_PROTOCOL("-");
        }
        if(flag & (1 << (i * 2 + 1))) {
            SERIAL_PROTOCOL(" ");
            SERIAL_PROTOCOL((char)(88 + i)); // char 88 = X
            SERIAL_PROTOCOL("O");
        }
    }
    SERIAL_PROTOCOL("\n");
}

void inline log_debug_fsr(float avg[3], float std[3] = NULL) {
    SERIAL_PROTOCOL("DEBUG FSR ");
    for(int i=0;i<3;i++) {
        SERIAL_PROTOCOL(avg[i]);
        SERIAL_PROTOCOL(" ");
    }

    if(std) {
        for(int i=0;i<3;i++) {
            SERIAL_PROTOCOL(std[i]);
            SERIAL_PROTOCOL(" ");
        }
    }
    SERIAL_PROTOCOL("\n");
}

void inline log_debug_zprobe_retry(int count) {
    SERIAL_ECHO("DEBUG: Retry= ");
    SERIAL_ECHO(count);
    SERIAL_PROTOCOL("\n");
}
