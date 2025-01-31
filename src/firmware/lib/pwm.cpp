#include "pwm.hpp"
#include "hardware/clocks.h"
#include <stdio.h>

PWM::PWM(uint pin, uint slice_num) {
    this->pin = pin;
    this->slice_num = slice_num;
    this->freq = 0;
    this->duty_cycle = 0;
    this->duty_cycle_us = 0;
    this->div = 0;
    this->resolution_us = 1;
}

void PWM::init() {
    gpio_init(this->pin);
    gpio_set_function(this->pin, GPIO_FUNC_PWM);
    this->resolution_us = 1;
    // set the clock divider to have n microsecond resolution
    uint32_t systemClock = clock_get_hz(clk_sys);
    this->div = (float)systemClock / (1000000.0 / this->resolution_us);
    pwm_set_clkdiv(this->slice_num, (float)div);
}

void PWM::set_resolution_us(uint16_t resolution_us){
    this->resolution_us = resolution_us;
    // set the clock divider to have n microsecond resolution
    uint32_t systemClock = clock_get_hz(clk_sys);
    this->div = (float)systemClock / (1000000.0 / this->resolution_us);
    pwm_set_clkdiv(this->slice_num, (float)div);
}

void PWM::set_freq(uint freq) {
    this->freq = freq;
    this->resolution_us = 1;
    // since we want n microsecond resolution, we need to set the wrap to 1/freq * 1e6 / resolution_us
    float wrap_float = (float)(1000000.0 / this->resolution_us) / (float)freq;
    this->wrap = (uint16_t)wrap_float;
    // decrease the resolution until wrap doesn't exceed 65535
    while (wrap_float > 65535) {
        this->resolution_us++;
        set_resolution_us(this->resolution_us);
        wrap_float = (float)(1000000.0 / this->resolution_us) / (float)freq;
        this->wrap = (uint16_t)wrap_float;
    }
    set_resolution_us(this->resolution_us);
    pwm_set_wrap(this->slice_num, this->wrap);
}

void PWM::set_duty_cycle(uint duty_cycle) {
    this->duty_cycle = duty_cycle;
    this->level = (uint16_t)((float)this->wrap * ((float)duty_cycle / 100.0));
    pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), level);
}

void PWM::set_duty_cycle_us(uint duty_cycle_us) {
    this->duty_cycle_us = duty_cycle_us;
    pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), duty_cycle_us / this->resolution_us);
}

void PWM::pause() {
    this->pause_state = true;
    pwm_set_chan_level(this->slice_num, pwm_gpio_to_channel(this->pin), 0);
}

void PWM::resume() {
    this->pause_state = false;
    set_duty_cycle(this->duty_cycle);
}

void PWM::set_enabled(bool enabled) {
    this->enabled = enabled;
    pwm_set_enabled(this->slice_num, this->enabled);
}
