#ifndef LSM6DSOX_HPP
#define LSM6DSOX_HPP

#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"
#include <stdio.h>

// LSM6DS Sensor Register Definitions
#define LSM6DS_I2CADDR 0x6A // Default I2C address of the LSM6DS sensor
#define LSM6DS_CHIP_ID 0x0F // Chip ID register, used to verify sensor identity

// Main Configuration Registers
#define LSM6DS_CTRL1_XL 0x10 // Accelerometer configuration register
#define LSM6DS_CTRL2_G 0x11  // Gyroscope configuration register
#define LSM6DS_CTRL3_C 0x12  // Main control register for global settings
#define LSM6DS_CTRL7_G 0x16  // Gyroscope high pass filter settings
#define LSM6DS_CTRL8_XL 0x17 // Accelerometer high and low pass filter settings
#define LSM6DS_CTRL9_XL 0x18 // Additional accelerometer control settings
#define LSM6DS_CTRL10_C 0x19 // Additional main control register

// Interrupt Configuration Registers
#define LSM6DS_INT1_CTRL 0x0D // Interrupt control for INT1 pin
#define LSM6DS_INT2_CTRL 0x0E // Interrupt control for INT2 pin
#define LSM6DS_MD1_CFG 0x5E   // Routing of functions to INT1 pin

// Sensor Output Registers
#define LSM6DS_STATUS_REG                                                      \
    0X1E // Status register indicating new data availability
#define LSM6DS_OUT_TEMP_L 0x20 // Low byte of temperature output data
#define LSM6DS_OUTX_L_G 0x22   // Low byte of gyroscope X-axis output data
#define LSM6DS_OUTX_L_A 0x28   // Low byte of accelerometer X-axis output data

// Feature Configuration Registers
#define LSM6DS_FUN_CFG_ACC 0x01 // Access register for embedded functions
#define LSM6DS_TAP_CFG 0x58     // Configuration for tap and pedometer functions
#define LSM6DS_WAKEUP_SRC 0x1B  // Source of wakeup event
#define LSM6DS_WAKEUP_THS 0x5B  // Threshold for wakeup event detection
#define LSM6DS_WAKEUP_DUR 0x5C  // Duration settings for wakeup and sleep events

// Other Registers
#define LSM6DS_STEPCOUNTER 0x4B // 16-bit step counter register

// Accelerometer Output ranges
enum class LSM6DSAccelRange {
    Range2G = 0, // ±2g range, maximum sensitivity, low maximum acceleration
    Range4G = 2, // ±4g range, high sensitivity
    Range8G = 3, // ±8g range, medium sensitivity
    Range16G = 1 // ±16g range, lowest sensitivity, high maximum acceleration
};               // ST really outdid themselves with the order on this one

// Gyroscope Output ranges
enum class LSM6DSGyroRange {
    Range125DPS = 0b001,  // ±125 degrees per second, very fine motion sensing
    Range250DPS = 0b000,  // ±250 degrees per second, fine motion sensing
    Range500DPS = 0b010,  // ±500 degrees per second, medium motion sensitivity
    Range1000DPS = 0b100, // ±1000 degrees per second, coarse motion sensitivity
    Range2000DPS =
        0b110 // ±2000 degrees per second, very coarse motion sensitivity
};

// Accelerometer Output Data Rates
enum class LSM6DSDataRate {
    Shutdown = 0, // No output data, sensor is in power down mode
    Rate12_5Hz,   // 12.5 Hz output data rate, very low power consumption
    Rate26Hz,     // 26 Hz output data rate, low power consumption
    Rate52Hz,     // 52 Hz output data rate, moderate power consumption
    Rate104Hz,    // 104 Hz output data rate, standard power consumption
    Rate208Hz,    // 208 Hz output data rate, higher power consumption
    Rate416Hz,    // 416 Hz output data rate, high power consumption
    Rate833Hz,    // 833 Hz output data rate, very high power consumption
    Rate1_66kHz,  // 1.66 kHz output data rate, ultra high power consumption
    Rate3_33kHz,  // 3.33 kHz output data rate, extreme power consumption
    Rate6_66kHz   // 6.66 kHz output data rate, maximum power consumption
};

// Filter bandwidths (high pass)
enum class LSM6DSFilter {
    ODRDiv4 = 0, // Bandwidth divided by 4
    ODRDiv10,    // Bandwidth divided by 10
    ODRDiv20,    // Bandwidth divided by 20
    ODRDiv45,    // Bandwidth divided by 45
    ODRDiv100,   // Bandwidth divided by 100
    ODRDiv200,   // Bandwidth divided by 200
    ODRDiv400,   // Bandwidth divided by 400
    ODRDiv800    // Bandwidth divided by 800
};

static const float accelRangeLookup[] = {2.0, 16.0, 4.0,
                                         8.0}; // for LSM6DSAccelRange
static const float gyroRangeLookup[] = {250.0, 125.0, 500.0, 1000.0,
                                        2000.0}; // for LSM6DSGyroRange

class LSM6DSOX {
  public:
    bool init(i2c_inst_t *i2c, uint8_t addr, uint8_t sda_pin, uint8_t scl_pin);

    void setAccelConf(LSM6DSDataRate rate, LSM6DSAccelRange range);
    void setGyroConf(LSM6DSDataRate rate, LSM6DSGyroRange range);
    void setFilter(LSM6DSFilter filter);
    void selectFilter(bool enable);
    void setGyroOffsets(float x, float y, float z);

    int readGyro(float *x, float *y, float *z);
    LSM6DSDataRate getGyroDataRate();
    LSM6DSGyroRange getGyroRange();
    bool gyroHasData();

    int readAccel(float *x, float *y, float *z);
    int readAccelMS(float *x, float *y, float *z);
    LSM6DSDataRate getAccelDataRate();
    LSM6DSAccelRange getAccelRange();
    bool accelHasData();

    float readTemp();

    uint8_t readRegister(i2c_inst_t *i2c, uint8_t addr, uint8_t reg);
    void writeRegister(i2c_inst_t *i2c, uint8_t addr, uint8_t reg,
                       uint8_t data);

  private:
    float offsetGyroX = 0.0f;
    float offsetGyroY = 0.0f;
    float offsetGyroZ = 0.0f;
    i2c_inst_t *i2c;
    uint8_t address;
    LSM6DSAccelRange accelRange;
    LSM6DSDataRate accelDataRate;
    LSM6DSGyroRange gyroRange;
    LSM6DSDataRate gyroDataRate;
};

#endif // LSM6DSOX_HPP