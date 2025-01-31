#ifndef HW_PWM_H
#define HW_PWM_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class PWM {
    public:
        PWM(uint pin, uint slice_num);
        void init();
        void set_freq(uint freq);
        void set_duty_cycle(uint duty_cycle);
        void set_duty_cycle_us(uint duty_cycle_us);
        void set_enabled(bool enabled);
        void pause();
        void resume();

        uint pin;
        uint slice_num;
        uint freq;
        uint duty_cycle;
        uint duty_cycle_us;
        float div;
        uint16_t wrap;
        uint16_t level;
        uint16_t resolution_us;
        bool enabled;
        bool pause_state;
        void set_resolution_us(uint16_t resolution_us);

};

#endif // HW_PWM_H
