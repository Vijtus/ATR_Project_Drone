#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "string.h"
#include "pwm.pio.h"

#include "lib/bt_telem.hpp"
#include "lib/ibus_rx.hpp"
#include "lib/pwm.hpp"
#include "lib/lsm6dsox.hpp"
#include "lib/utils.hpp"
#include "lib/ewma_filter.hpp"
#include "lib/pid.hpp"

// CONTROLS on the remote
/*
┌───────────────────────────────────────┐
│           ▲                  ▲        │
│        CH3│               CH1│        │
│           │                  │        │
│        ┌──┼──┐            ┌──┼──┐     │
│     CH4│  │  │         CH0│  │  │     │
│    ◄───┼──┼──┼───►    ◄───┼──┼──┼───► │
│ Yaw    │  │  │      Roll  │  │  │     │
│        └──┼──┘            └──┼──┘     │
│           │                  │        │
│           │                  │        │
│           ▼                  ▼        │
│        Throttle           Pitch       │
└───────────────────────────────────────┘
*/
// channel 5 unused 3 position switch
// channel 6 estop switch (SWA)


// we will be flying this drone like in the simulation
// the drone looks like this:
/*
    X
    1
    |
Y 3- -4
    |
    2

*/
// the motors have the following directions:
// 1: CCW   (counter clockwise, front of the drone)
// 2: CCW   (counter clockwise, back of the drone)
// 3: CW    (clockwise, left of the drone)
// 4: CW    (clockwise, right of the drone)
// our pitch forward command means to rotate along the Y axis and move in the direction of X
// this is unlike the traditional consumer drone way of pitching both motor 1 and 3 down
// this is because we want to keep the equations of motion the same as in the simulation
// as well as simplifying the number of values influencing each motor speed
// consequently, our IMU board has to be mounted in a way to have it's X and Y axis facing the same way as the drone
// so our control board sits at a 45 degree angle to the drone
// this can be changed later and it should be pretty easy

// ===== DEFINES =====
#define CONTROL_LOOP_FREQ 200           // Hz, the frequency at which the control loop runs
#define POINTS_BIAS_CALIBRATION 1024    // the number of points to use for the gyroscope bias calibration


// ===== ARMING AND DISARMING =====
// arming is done as follows:
// put the left stuck in the bottom left corner and wait for ARM_DISARM_TIME
// the drone will start spinning the motors slowly
// to disarm put the left stick in the bottom right corner and wait for ARM_DISARM_TIME
// the drone will stop spinning the motors

#define ARM_DISARM_TIME 2000000         // us, the time the user has to hold the sticks in the arm/disarm position to arm/disarm the drone

#define ARM_CH4_LEVEL_MIN 950           // lower bound for arming sequence on channel 4
#define ARM_CH4_LEVEL_MAX 1050          // upper bound for arming sequence on channel 4
#define DISARM_CH4_LEVEL_MIN 1900       // lower bound for disarming sequence on channel 4 (no need for upper bound, we just need to exceed the min value)

#define ARM_DISARM_CH3_LEVEL_MIN 900   // lower bound for arming/disarming sequence on channel 3
#define ARM_DISARM_CH3_LEVEL_MAX 1100  // upper bound for arming/disarming sequence on channel 3

// ===== ESC CALIBRATION =====
// ESC calibration is done as follows:
// put the left stick into top right corner and connect power to the drone
// the pico LED will blink rapidly
// move the left stick into the center, but keep it in the top
// the pico LED will blink slowly
// move the stick down the center
// the pico LED will turn off and the ESCs will beep
// the ESCs are now calibrated

// we don't need max values because it just needs to exceed the min value
#define ESC_CALIBRATE_CH3_LEVEL_MIN 1900    // lower bound for ESC calibration sequence on channel 3
#define ESC_CALIBRATE_CH4_MIN 1900          // lower bound for ESC calibration sequence on channel 4

// ===== ESCS =====
#define MOTOR_ARM_SPEED 1080            // us, the speed at which the motors spin up when armed and still on the ground

#define MOTOR_MIN MOTOR_ARM_SPEED                  // us, the minimum duty cycle for the motors
#define MOTOR_MAX 2000                  // us, the maximum duty cycle for the motors


// ===== DEADZONES =====
#define CH1_DEADZONE 10                 // the deadzone for channel 1
#define CH1_DEADZONE_POINT 1500         // the point at which the deadzone should be applied
#define CH2_DEADZONE 10                 // the deadzone for channel 2
#define CH2_DEADZONE_POINT 1500         // the point at which the deadzone should be applied
#define CH3_DEADZONE 20                 // the deadzone for channel 3
#define CH3_DEADZONE_POINT 1000         // the point at which the deadzone should be applied
#define CH4_DEADZONE 10                 // the deadzone for channel 4
#define CH4_DEADZONE_POINT 1500         // the point at which the deadzone should be applied

// ===== MISCELLANEOUS =====
#define THROTTLE_NORM_MAX 0.6           // the maximum normalized throttle value, this is needed to we don't saturate the motor and so that the PID retains some control
#define THROTTLE_NORM_MIN 0.07          // the minimum normalized throttle value, this is the minimal speed at which the motors still spin

#define ESTOP_CHANNEL 6                 // the channel that triggers the emergency stop
#define ESTOP_LEVEL 1100                // the level above which the emergency stop is triggered

#define IMU_DPS_RANGE 1000              // the range of the gyroscope in degrees per second

#define CONTROL_FACTOR_PITCH 0.2              // the factor by which the remote input is multiplied before being fed into the PID controllers
#define CONTROL_FACTOR_ROLL 0.2               // the factor by which the remote input is multiplied before being fed into the PID controllers
#define CONTROL_FACTOR_YAW 0.8                // the factor by which the remote input is multiplied before being fed into the PID controllers

// ===== PID =====
#define PID_PITCH_KP 1//0.0010//0.0005                // the proportional gain for the pitch PID
#define PID_PITCH_KI 0//0.0                // the integral gain for the pitch PID
#define PID_PITCH_KD 0.0165//0.0000165//0.00001125              // the derivative gain for the pitch PID

#define PID_ROLL_KP PID_PITCH_KP        // the proportional gain for the roll PID
#define PID_ROLL_KI PID_PITCH_KI        // the integral gain for the roll PID
#define PID_ROLL_KD PID_PITCH_KD        // the derivative gain for the roll PID

#define PID_YAW_KP 0.3//0.0003                  // the proportional gain for the yaw PID
#define PID_YAW_KI 0.02//0.00002                // the integral gain for the yaw PID
#define PID_YAW_KD 0//0.0                  // the derivative gain for the yaw PID

#define MAX_PITCH_ANGLE 0.3             // the maximum pitch angle in degrees
#define MAX_ROLL_ANGLE 0.3              // the maximum roll angle in degrees
#define MAX_YAW_RATE 0.5                // the maximum yaw rate in degrees per second

// ===== CONSTANTS =====
const uint sda_pin = 12;
const uint scl_pin = 13;

const uint ss_pin = 14;
const uint irq_pin = 15;

const uint telem_pin_tx = 0;
const uint telem_pin_rx = 1;

const uint ibus_pin_tx = 4;
const uint ibus_pin_rx = 5;

const uint esc_pin_1 = 8;
const uint esc_pin_2 = 10;
const uint esc_pin_3 = 12;
const uint esc_pin_4 = 14;

const uint imu_irq_pin = 3;
const uint imu_sda_pin = 16;
const uint imu_scl_pin = 17;

static const uint led_pin_pico = 25;
static const uint led_pin_board = 6;

// ===== VARIABLES =====
// states
enum State {
    ESTOP = -2,     // emergency stop state, we are in the process of disabling the motors, simmilair to the error state
    ERROR,          // error state, we cannot recover from this
    DISARMED,       // disarmed state, we are waiting for the user to arm the drone
    ARMED,          // armed state, we are ready to fly/flying
    CALIBRATING     // calibrating state, we are calibrating the ESCs, and profiling the IMU noise
} state;

// BT telemetry flags
bool should_sent = false;
bool sending = false;

// ibus channels
uint16_t ibus_channels[14];
float norm_roll, norm_pitch, norm_throttle, norm_yaw;

// led states 
static bool pico_led_state = false;
static bool board_led_state = false;
uint32_t last_led_time = 0;

// imu data and calibration
float ax, ay, az;
float gx, gy, gz;

float norm_gx, norm_gy, norm_gz;

float gx_bias, gy_bias, gz_bias;
float bias_buff[3][POINTS_BIAS_CALIBRATION];

// imu filtering
float decayFactor = 0.5;    // lower = more filtering, higher = less filtering
EWMAFilter gxFilter(decayFactor, 0.0);
EWMAFilter gyFilter(decayFactor, 0.0);
EWMAFilter gzFilter(decayFactor, 0.0);

// PID parameters
float pid_pitch_kp = PID_PITCH_KP;
float pid_pitch_ki = PID_PITCH_KI;
float pid_pitch_kd = PID_PITCH_KD;

float pid_roll_kp = PID_ROLL_KP;
float pid_roll_ki = PID_ROLL_KI;
float pid_roll_kd = PID_ROLL_KD;

float pid_yaw_kp = PID_YAW_KP;
float pid_yaw_ki = PID_YAW_KI;
float pid_yaw_kd = PID_YAW_KD;

float pid_time_step = 1.0 / CONTROL_LOOP_FREQ;

float integral_limit = 10.0;

PIDController pid_pitch(pid_pitch_kp, pid_pitch_ki, pid_pitch_kd, pid_time_step, integral_limit);
PIDController pid_roll(pid_roll_kp, pid_roll_ki, pid_roll_kd, pid_time_step, integral_limit);
PIDController pid_yaw(pid_yaw_kp, pid_yaw_ki, pid_yaw_kd, pid_time_step, integral_limit);

float pitch_pid_command = 0.0;
float roll_pid_command = 0.0;
float yaw_pid_command = 0.0;

// motor speeds
float motor_1_speed = 0.0;
float motor_2_speed = 0.0;
float motor_3_speed = 0.0;
float motor_4_speed = 0.0;

uint motor_1_speed_us = 0;
uint motor_2_speed_us = 0;
uint motor_3_speed_us = 0;
uint motor_4_speed_us = 0;

// ===== FUNCTIONS =====
// core 1 is used for non critical tasks, such as sending telemetry
void core1(){
    while(true){
        // check if we should send the telemetry
        if(should_sent && !sending){
            sending = true;
            buffer_send();
            sending = false;
            should_sent = false;
        }
    }
}

// core 0 is used for critical tasks, and control
int main() {
    stdio_init_all();

    // set the satate to disarmed
    state = DISARMED;

    // initialize the bluetooth telemetry
    bt_init(uart0, telem_pin_tx, telem_pin_rx);

    // launch core 1
    multicore_launch_core1(core1);

    // initialize the uart for ibus
    IBUS_Decode ibus;
    ibus.init(uart1, ibus_pin_tx, ibus_pin_rx);

    // initialize the pwm pins for the ESCs nad the ESCs themselves
    PWM esc1(esc_pin_1, pwm_gpio_to_slice_num(esc_pin_1));
    PWM esc2(esc_pin_2, pwm_gpio_to_slice_num(esc_pin_2));
    PWM esc3(esc_pin_3, pwm_gpio_to_slice_num(esc_pin_3));
    PWM esc4(esc_pin_4, pwm_gpio_to_slice_num(esc_pin_4));

    esc1.init();
    esc2.init();
    esc3.init();
    esc4.init();

    esc1.set_freq(50);
    esc2.set_freq(50);
    esc3.set_freq(50);
    esc4.set_freq(50);

    esc1.set_enabled(false);
    esc2.set_enabled(false);
    esc3.set_enabled(false);
    esc4.set_enabled(false);

    // initialize the imu
    LSM6DSOX imu;
    imu.init(i2c0, LSM6DS_I2CADDR, imu_sda_pin, imu_scl_pin);

    imu.setAccelConf(LSM6DSDataRate::Rate416Hz, LSM6DSAccelRange::Range16G);
    imu.setGyroConf(LSM6DSDataRate::Rate416Hz, LSM6DSGyroRange::Range1000DPS);

    imu.setFilter(LSM6DSFilter::ODRDiv800);
    imu.selectFilter(false);

    sleep_ms(3000);

    for (int i = 0; i < POINTS_BIAS_CALIBRATION; i++) {
        while(!imu.gyroHasData()){
            sleep_ms(1);
        }
        imu.readGyro(&gx, &gy, &gz);
        bias_buff[0][i] = gx;
        bias_buff[1][i] = gy;
        bias_buff[2][i] = gz;
        sleep_ms(1);
    }
    // compute the average bias for each axis
    for (int i = 0; i < POINTS_BIAS_CALIBRATION; i++) {
        gx_bias += bias_buff[0][i];
        gy_bias += bias_buff[1][i];
        gz_bias += bias_buff[2][i];
    }
    gx_bias /= POINTS_BIAS_CALIBRATION;
    gy_bias /= POINTS_BIAS_CALIBRATION;
    gz_bias /= POINTS_BIAS_CALIBRATION;

    // set the bias

    imu.setGyroOffsets(gx_bias, gy_bias, gz_bias);

    printf("gx_bias: %f\n", gx_bias);
    printf("gy_bias: %f\n", gy_bias);
    printf("gz_bias: %f\n", gz_bias);    

    // imu.setAccelCompositeInput(true);

    // initialize the led pins
    gpio_init(led_pin_pico);
    gpio_set_dir(led_pin_pico, GPIO_OUT);
    gpio_put(led_pin_pico, 1); 

    gpio_init(led_pin_board);
    gpio_set_dir(led_pin_board, GPIO_OUT);
    gpio_put(led_pin_board, 1);

    // TODO debug pin, used to measure the timing of stuff, remove later
    gpio_init(15);
    gpio_set_dir(15, GPIO_OUT);
    gpio_set_slew_rate(15, GPIO_SLEW_RATE_FAST);
    gpio_put(15, 0);


    // wait for a moment for everything to settle
    sleep_ms(100);

    // wait for ibus to decode a non zero packet

    while(true){
        ibus.decode();
        ibus.get_all_channels(ibus_channels);
        if(ibus_channels[0] != 0){
            break;
        }
        gpio_put(led_pin_board, board_led_state);
        sleep_ms(100);
        board_led_state = !board_led_state;
    }

    for (int i = 0; i < 14; i++) {
        printf("%d ", ibus_channels[i]);
    }
    printf("\n");

    // check if we are in ESC calibration mode
    if(ibus_channels[2] > ESC_CALIBRATE_CH3_LEVEL_MIN && ibus_channels[3] > ESC_CALIBRATE_CH4_MIN){
        printf("ESC calibration mode\n");
        // we are in ESC calibration mode
        // blink the pico LED rapidly
        last_led_time = time_us_32();
        while(true){
            ibus.decode();
            ibus.get_all_channels(ibus_channels);

            if(time_us_32() - last_led_time > 100000){
                last_led_time = time_us_32();
                pico_led_state = !pico_led_state;
            }
            // check if channel 4 is no longer in the calibration range
            if(ibus_channels[3] < ESC_CALIBRATE_CH3_LEVEL_MIN){
                break;
            }
            gpio_put(led_pin_pico, pico_led_state);

            esc1.set_duty_cycle_us(ibus_channels[2]);
            esc2.set_duty_cycle_us(ibus_channels[2]);
            esc3.set_duty_cycle_us(ibus_channels[2]);
            esc4.set_duty_cycle_us(ibus_channels[2]);

        }
        printf("ESC calibration begin\n");

        esc1.set_enabled(true);
        esc2.set_enabled(true);
        esc3.set_enabled(true);
        esc4.set_enabled(true);

        sleep_ms(10);

        // a lot of repeating code I know
        uint32_t first_in_deadzone_time = 0;
        uint32_t in_deadzone_time = 0;
        // blink the led slowly and pipe out the esc value
        while (true)
        {
            ibus.decode();
            ibus.get_all_channels(ibus_channels);

            if(time_us_32() - last_led_time > 100000){
                last_led_time = time_us_32();
                pico_led_state = !pico_led_state;
            }
            // check if channel 3 reached the deadzone for a while
            if(ibus_channels[2] < (CH3_DEADZONE_POINT + (CH3_DEADZONE/2)) && ibus_channels[2] > (CH3_DEADZONE_POINT - (CH3_DEADZONE/2))){
                if(first_in_deadzone_time == 0){
                    first_in_deadzone_time = time_us_32();
                }
                in_deadzone_time = time_us_32();
            }else{
                first_in_deadzone_time = 0;
                in_deadzone_time = 0;
            }
            gpio_put(led_pin_pico, pico_led_state);

            esc1.set_duty_cycle_us(ibus_channels[2]);
            esc2.set_duty_cycle_us(ibus_channels[2]);
            esc3.set_duty_cycle_us(ibus_channels[2]);
            esc4.set_duty_cycle_us(ibus_channels[2]);

            if(in_deadzone_time - first_in_deadzone_time > 1000000){
                break;
            }

        }
        gpio_put(led_pin_pico, true);

        // wait for the escs to finish beeping
        sleep_ms(5000);

        esc1.set_enabled(false);
        esc2.set_enabled(false);
        esc3.set_enabled(false);
        esc4.set_enabled(false);

        esc1.set_duty_cycle_us(1000);
        esc2.set_duty_cycle_us(1000);
        esc3.set_duty_cycle_us(1000);
        esc4.set_duty_cycle_us(1000);

        printf("ESC calibration end\n");
        gpio_put(led_pin_pico, false);
    }

    esc1.set_duty_cycle_us(1000);
    esc2.set_duty_cycle_us(1000);
    esc3.set_duty_cycle_us(1000);
    esc4.set_duty_cycle_us(1000);

    esc1.set_enabled(true);
    esc2.set_enabled(true);
    esc3.set_enabled(true);
    esc4.set_enabled(true);

    sleep_ms(100);

    uint32_t last_loop_time = time_us_32();

    // main loop
    while (true)
    {   
        // code that needs to always execute

        // decode the ibus data and update the channels        
        ibus.decode();
        ibus.get_all_channels(ibus_channels);
        norm_roll = normalize(ibus_channels[0], 1000, 2000, -1.0, 1.0);
        norm_pitch = normalize(ibus_channels[1], 1000, 2000, -1.0, 1.0);
        norm_throttle = normalize(ibus_channels[2], 1000, 2000, THROTTLE_NORM_MIN, THROTTLE_NORM_MAX);  // we can't have negative throttle
        norm_yaw = normalize(ibus_channels[3], 1000, 2000, -1.0, 1.0);

        // scale the control inputs
        norm_roll *= CONTROL_FACTOR_ROLL;
        norm_pitch *= CONTROL_FACTOR_PITCH;
        norm_yaw *= CONTROL_FACTOR_YAW;

        // for (int i = 0; i < 14; i++) {
        //     printf("%d ", ibus_channels[i]);
        // }
        // printf("\n");

        // check for ibus timeout
        if(ibus.timed_out()){
            // we have lost connection to the remote
            state = ESTOP;
            printf("IBUS timeout\n");
        }

        // check if estop channel is triggered
        if (ibus_channels[ESTOP_CHANNEL - 1] > ESTOP_LEVEL) {
            state = ESTOP;
            printf("ESTOP channel triggered\n");
        }

        // check if there is new imu data
        if(imu.gyroHasData()){
            imu.readGyro(&gx, &gy, &gz);
        }
        // we don't need the accelerometer data for now
        // if(imu.accelHasData()){
        //     imu.readAccelMS(&ax, &ay, &az);
        // }

        // filter accelerometer data
        gxFilter.addData(gx);
        gyFilter.addData(gy);
        gzFilter.addData(gz);

        norm_gx = normalize(gxFilter.getFilteredData(), -IMU_DPS_RANGE, IMU_DPS_RANGE, -1.0, 1.0);
        norm_gy = normalize(gyFilter.getFilteredData(), -IMU_DPS_RANGE, IMU_DPS_RANGE, -1.0, 1.0);
        norm_gz = normalize(gzFilter.getFilteredData(), -IMU_DPS_RANGE, IMU_DPS_RANGE, -1.0, 1.0);

        //printf("gx:%f, gy:%f, gz:%f, fgx:%f, fgy:%f, fgz:%f\n", gx, gy, gz, gxFilter.getFilteredData(), gyFilter.getFilteredData(), gzFilter.getFilteredData());

        //printf("gx:%.3f, gy:%.3f, gz:%.3f - ax:%.3f, ay:%.3f, az:%.3f\n", gx, gy, gz, ax, ay, az);
        //printf("%f, %f, %f\n", ax, ay, az);

        // code dependent on the state
        switch(state){
            case ESTOP:
                // kill all the motors
                esc1.set_enabled(false);
                esc2.set_enabled(false);
                esc3.set_enabled(false);
                esc4.set_enabled(false);
                // flash both leds
                last_led_time = time_us_32();
                pico_led_state = false;
                while (true)
                {
                    if(time_us_32() - last_led_time > 100000){
                        last_led_time = time_us_32();
                        pico_led_state = !pico_led_state;
                        //printf("ESTOP\n");
                    }
                    gpio_put(led_pin_pico, pico_led_state);
                    gpio_put(led_pin_board, pico_led_state);
                }
                break;
            case ERROR:
                // TODO
                break;
            case DISARMED:

                // send 0 throttle to the motors
                esc1.set_duty_cycle_us(1000);
                esc2.set_duty_cycle_us(1000);
                esc3.set_duty_cycle_us(1000);
                esc4.set_duty_cycle_us(1000);

                // ===== CHECK IF USER WANTS TO ARM =====

                static uint32_t first_in_arm_position_time = 0;
                static uint32_t in_arm_position_time = 0;

                // check if the user is trying to arm the drone
                if(ibus_channels[3] > ARM_CH4_LEVEL_MIN && ibus_channels[3] < ARM_CH4_LEVEL_MAX && ibus_channels[2] > ARM_DISARM_CH3_LEVEL_MIN && ibus_channels[2] < ARM_DISARM_CH3_LEVEL_MAX) {

                    // Check if channel 4 is in the deadzone
                    if(ibus_channels[3] > ARM_CH4_LEVEL_MIN && ibus_channels[3] < ARM_CH4_LEVEL_MAX) {
                        if(first_in_arm_position_time == 0){
                            first_in_arm_position_time = time_us_32();
                        }
                        in_arm_position_time = time_us_32();
                    } else {
                        first_in_arm_position_time = 0;
                        in_arm_position_time = 0;
                    }

                    // Check if channel 3 is in the deadzone
                    if(ibus_channels[2] > ARM_DISARM_CH3_LEVEL_MIN && ibus_channels[2] < ARM_DISARM_CH3_LEVEL_MAX) {
                        if(first_in_arm_position_time == 0){
                            first_in_arm_position_time = time_us_32();
                        }
                        in_arm_position_time = time_us_32();
                    } else {
                        first_in_arm_position_time = 0;
                        in_arm_position_time = 0;
                    }

                    // Check if the user has held the sticks in the arm position for long enough
                    if(in_arm_position_time - first_in_arm_position_time > ARM_DISARM_TIME) {
                        // Arm the drone
                        state = ARMED;

                        // Spin up the motors
                        esc1.set_enabled(true);
                        esc2.set_enabled(true);
                        esc3.set_enabled(true);
                        esc4.set_enabled(true);

                        // reset the variables
                        first_in_arm_position_time = 0;
                        in_arm_position_time = 0;

                        printf("ARMED\n");

                    }
                }
                // continuously reset the PID controllers so that they don't accumulate anything while on the ground
                pid_pitch.reset();
                pid_roll.reset();
                pid_yaw.reset();

                //TODO remove
                // print the filtered gyro data
                //printf("gx:%f, gy:%f, gz:%f, fgx:%f, fgy:%f, fgz:%f\n", gx, gy, gz, gxFilter.getFilteredData(), gyFilter.getFilteredData(), gzFilter.getFilteredData());
                

                break;
            case ARMED:

                // ===== CHECK IF USER WANTS TO DISARM =====

                // same as above but for disarming
                static uint32_t first_in_disarm_position_time = 0;
                static uint32_t in_disarm_position_time = 0;

                // check if user is trying to disarm the drone
                if(ibus_channels[3] > DISARM_CH4_LEVEL_MIN && ibus_channels[2] > ARM_DISARM_CH3_LEVEL_MIN && ibus_channels[2] < ARM_DISARM_CH3_LEVEL_MAX) {

                    // Check if channel 4 is in the deadzone
                    if(ibus_channels[3] > DISARM_CH4_LEVEL_MIN) {
                        if(first_in_disarm_position_time == 0){
                            first_in_disarm_position_time = time_us_32();
                        }
                        in_disarm_position_time = time_us_32();
                    } else {
                        first_in_disarm_position_time = 0;
                        in_disarm_position_time = 0;
                    }

                    // Check if channel 3 is in the deadzone
                    if(ibus_channels[2] > ARM_DISARM_CH3_LEVEL_MIN && ibus_channels[2] < ARM_DISARM_CH3_LEVEL_MAX) {
                        if(first_in_disarm_position_time == 0){
                            first_in_disarm_position_time = time_us_32();
                        }
                        in_disarm_position_time = time_us_32();
                    } else {
                        first_in_disarm_position_time = 0;
                        in_disarm_position_time = 0;
                    }

                    // Check if the user has held the sticks in the disarm position for long enough
                    if(in_disarm_position_time - first_in_disarm_position_time > ARM_DISARM_TIME) {
                        // Disarm the drone
                        state = DISARMED;

                        // Spin down the motors
                        esc1.set_duty_cycle_us(1000);
                        esc2.set_duty_cycle_us(1000);
                        esc3.set_duty_cycle_us(1000);
                        esc4.set_duty_cycle_us(1000);

                        // reset the variables
                        first_in_disarm_position_time = 0;
                        in_disarm_position_time = 0;

                        printf("DISARMED\n");
                    }
                }

                // ===== CONTROL LOOP =====

                // push desired and actual angles to the PID controllers
                // due to our drone configuration, we need to swap the X and Y axes for the pitch and roll rotational axes
                pitch_pid_command = pid_pitch.calculate(norm_pitch, norm_gy);
                roll_pid_command = pid_roll.calculate(norm_roll, norm_gx);
                yaw_pid_command = pid_yaw.calculate(norm_yaw, norm_gz);     

                // clamp the pid commands
                pitch_pid_command = clamp(pitch_pid_command, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
                roll_pid_command = clamp(roll_pid_command, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
                yaw_pid_command = clamp(yaw_pid_command, -MAX_YAW_RATE, MAX_YAW_RATE);           

                //printf("norm_pitch:%f,norm_roll:%f,norm_yaw:%f,", norm_pitch, norm_roll, norm_yaw);
                printf("pitch_pid_command:%f,roll_pid_command:%f,yaw_pid_command:%f,", pitch_pid_command, roll_pid_command, yaw_pid_command);

                // calculate the motor speeds
                motor_1_speed = norm_throttle + pitch_pid_command + yaw_pid_command;
                motor_2_speed = norm_throttle - pitch_pid_command + yaw_pid_command;
                motor_3_speed = norm_throttle - roll_pid_command - yaw_pid_command;
                motor_4_speed = norm_throttle + roll_pid_command - yaw_pid_command;

                printf("motor_1_speed:%f,motor_2_speed:%f,motor_3_speed:%f,motor_4_speed:%f\n", motor_1_speed, motor_2_speed, motor_3_speed, motor_4_speed);

                // convert motor speeds to duty cycles
                motor_1_speed_us = map(motor_1_speed, 0.0, 1.0, MOTOR_MIN, MOTOR_MAX);
                motor_2_speed_us = map(motor_2_speed, 0.0, 1.0, MOTOR_MIN, MOTOR_MAX);
                motor_3_speed_us = map(motor_3_speed, 0.0, 1.0, MOTOR_MIN, MOTOR_MAX);
                motor_4_speed_us = map(motor_4_speed, 0.0, 1.0, MOTOR_MIN, MOTOR_MAX);

                // set the motor speeds
                if(ibus_channels[0] < ARM_DISARM_CH3_LEVEL_MAX){
                    esc1.set_duty_cycle_us(MOTOR_ARM_SPEED);
                    esc2.set_duty_cycle_us(MOTOR_ARM_SPEED);
                    esc3.set_duty_cycle_us(MOTOR_ARM_SPEED);
                    esc4.set_duty_cycle_us(MOTOR_ARM_SPEED);
                }else{
                    esc1.set_duty_cycle_us(motor_1_speed_us);
                    esc2.set_duty_cycle_us(motor_2_speed_us);
                    esc3.set_duty_cycle_us(motor_3_speed_us);
                    esc4.set_duty_cycle_us(motor_4_speed_us);
                }

                //printf("%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n", motor_1_speed, motor_2_speed, motor_3_speed, motor_4_speed, motor_1_speed_us, motor_2_speed_us, motor_3_speed_us, motor_4_speed_us);

                break;
            case CALIBRATING:
                // TODO
                break;
        }
        



        
        // Debugging
        
        // for (int i = 0; i < 14; i++) {
        //     printf("%d ", ibus_channels[i]);
        // }
        // printf(" - %i\n", state);
        // if(ibus.timed_out()){
        //     printf("timed out\n");
        // }

        // sleep_ms(1000);
        // sleep_ms(1);

        // now we will pipe the first 4 channels to the ESCs
        // esc1.set_duty_cycle_us(ibus_channels[2]);
        // esc2.set_duty_cycle_us(ibus_channels[2]);
        // esc3.set_duty_cycle_us(ibus_channels[2]);
        // esc4.set_duty_cycle_us(ibus_channels[2]);

        // esc1.set_enabled(true);
        // esc2.set_enabled(true);
        // esc3.set_enabled(true);
        // esc4.set_enabled(true);
        
        // ===========================
        // testing the imu library

        // if (imu.accelHasData()){
        //     //imu.readAccel(&ax, &ay, &az);
        //     imu.readGyro(&gx, &gy, &gz);
        //     // printf("ax: %f, ay: %f, az: %f\n", ax, ay, az);
        //     // printf("gx: %f, gy: %f, gz: %f - ", gx, gy, gz);
        //     printf("%f,%f,%f\n", gx, gy, gz);
        // }

        // ===========================
        // testing the bt_telem library

        // char msg1[64] = {0};
        // char msg2[64] = {0};
        // char msg3[80] = {0};

        // sprintf(msg1, "ax: %f, ay: %f, az: %f", ax, ay, az);
        // sprintf(msg2, "gx: %f, gy: %f, gz: %f\n", gx, gy, gz);
        // sprintf(msg3, "ch0: %d, ch1: %d, ch2: %d, ch3: %d\n", channels[0], channels[1], channels[2], channels[3]);

        // if(!sending){
        //     buffer_clear();
        //     uint str_length = strlen(msg1);
        //     buffer_append(msg1, str_length);
        //     buffer_append(msg2, strlen(msg2));
        //     buffer_append(msg3, strlen(msg3));
        //     // buffer_send();
        //     should_sent = true;
        // }

        // printf("%d\n", time_us_32());
        // sleep_ms(100);

        uint32_t current_time = time_us_32();
        uint32_t loop_time = current_time - last_loop_time;
        uint32_t target_loop_time = 1000000/CONTROL_LOOP_FREQ;
        uint32_t sleep_time = target_loop_time - loop_time;
        if(loop_time > target_loop_time){
            printf("loop time exceeded\n");
        }else{
            sleep_us(sleep_time);
        }
        //printf("loop_time:%d, sleep_time:%d\n", loop_time, sleep_time);
        last_loop_time = time_us_32();
    }
    

}