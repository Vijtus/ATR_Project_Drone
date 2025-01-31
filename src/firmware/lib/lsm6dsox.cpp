#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "lsm6dsox.hpp"

uint8_t LSM6DSOX::readRegister(i2c_inst_t *i2c, uint8_t addr, uint8_t reg) {
    uint8_t buffer[1];
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 1, false);
    return buffer[0];
}

void LSM6DSOX::writeRegister(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}


bool LSM6DSOX::init(i2c_inst_t *i2c, uint8_t addr, uint8_t sda_pin, uint8_t scl_pin) {
    // Initialize I2C with the provided pins and set up the device
    // You might need to configure the I2C clock speed as well
    i2c_init(i2c, 400 * 1000); // 400kHz
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    this->i2c = i2c;
    this->address = addr;
    // try reading
    uint8_t rxdata;
    int ret = i2c_read_blocking(i2c, addr, &rxdata, 1, false);
    if(ret == PICO_ERROR_GENERIC){
        return false;
    }
    sleep_us(50);
    // reset the sensor
    rxdata = readRegister(i2c, addr, LSM6DS_CTRL3_C);
    rxdata |= 0x01;
    writeRegister(i2c, addr, LSM6DS_CTRL3_C, rxdata);
    bool reset = true;
    do{
        sleep_us(50);
        rxdata = readRegister(i2c, addr, LSM6DS_CTRL3_C);
        reset = (rxdata & 0x01) != 0;
    }while (reset);
    // enable Block Data Update (we are sampling manually)
    rxdata = readRegister(i2c, addr, LSM6DS_CTRL3_C);
    rxdata |= 0x40;
    writeRegister(i2c, addr, LSM6DS_CTRL3_C, rxdata);

    // setup some sensible defaults
    sleep_us(50);
    // set gyro to 104Hz, 2000dps
    writeRegister(i2c, addr, LSM6DS_CTRL2_G, 0x60);
    // set accel to 104Hz, 4g
    writeRegister(i2c, addr, LSM6DS_CTRL1_XL, 0x4A);
    // set power mode and bandwidth
    writeRegister(i2c, addr, LSM6DS_CTRL7_G, 0x00);
    sleep_us(50);

    return true;
}

void LSM6DSOX::setAccelConf(LSM6DSDataRate rate, LSM6DSAccelRange range) {
    uint8_t regValue = (uint8_t)rate << 4;
    regValue |= (uint8_t)range << 2;
    uint8_t buffer[2] = {LSM6DS_CTRL1_XL, regValue};
    i2c_write_blocking(i2c, address, buffer, 2, false);
    this->accelDataRate = rate;
    this->accelRange = range;
}

void LSM6DSOX::setGyroConf(LSM6DSDataRate rate, LSM6DSGyroRange range) {
    uint8_t regValue = (uint8_t)rate << 4;
    regValue |= (uint8_t)range << 1;
    uint8_t buffer[2] = {LSM6DS_CTRL2_G, regValue};
    i2c_write_blocking(i2c, address, buffer, 2, false);
    this->gyroDataRate = rate;
    this->gyroRange = range;
}

// function used to purely set the HPCF_XL_[2:0] bits in CTRL8_XL
void LSM6DSOX::setFilter(LSM6DSFilter filter) {
    uint8_t regValue = readRegister(i2c, address, LSM6DS_CTRL8_XL);
    regValue &= 0x1F;
    regValue |= (uint8_t)filter << 5;
    uint8_t buffer[2] = {LSM6DS_CTRL8_XL, regValue};
    i2c_write_blocking(i2c, address, buffer, 2, false);
}

// true = high pass, false = low pass
void LSM6DSOX::selectFilter(bool high_pass) {
    uint8_t regValue = readRegister(i2c, address, LSM6DS_CTRL8_XL);
    regValue &= 0xFB;
    regValue |= (high_pass ? 1 : 0) << 2;
    uint8_t buffer[2] = {LSM6DS_CTRL8_XL, regValue};
    i2c_write_blocking(i2c, address, buffer, 2, false);
    if (!high_pass){
        regValue = readRegister(i2c, address, LSM6DS_CTRL1_XL);
        regValue &= 0xFD;
        regValue |= 1 << 1;
        buffer[0] = LSM6DS_CTRL1_XL;
        buffer[1] = regValue;
        i2c_write_blocking(i2c, address, buffer, 2, false);
    }
}

int LSM6DSOX::readAccel(float *x, float *y, float *z) {
    uint8_t buffer[2];
    uint8_t complete_buffer[6];
    uint8_t reg = LSM6DS_OUTX_L_A;

    // read out 2 values at a time, incrementing the register address
    for(uint i = 0; i < 3; i++){
        if (i2c_write_timeout_us(i2c, address, &reg, 1, true, 1000) == PICO_ERROR_GENERIC){
            return 0;
        }
        if (i2c_read_timeout_us(i2c, address, buffer, 2, false, 1000) == PICO_ERROR_GENERIC){
            return 0;
        }
        complete_buffer[2*i] = buffer[0];
        complete_buffer[2*i+1] = buffer[1];
        reg += 2;
    }

    int16_t rawX = (complete_buffer[1] << 8) | complete_buffer[0];
    int16_t rawY = (complete_buffer[3] << 8) | complete_buffer[2];
    int16_t rawZ = (complete_buffer[5] << 8) | complete_buffer[4];

    uint range = (int)(this->accelRange);
    float scale = accelRangeLookup[range];

    *x = (float)rawX * scale / 32768.0;
    *y = (float)rawY * scale / 32768.0;
    *z = (float)rawZ * scale / 32768.0;
    return 1;
    
}

int LSM6DSOX::readAccelMS(float *x, float *y, float *z) {
    uint8_t buffer[2];
    uint8_t complete_buffer[6];
    uint8_t reg = LSM6DS_OUTX_L_A;

    // read out 2 values at a time, incrementing the register address
    for(uint i = 0; i < 3; i++){
        if (i2c_write_timeout_us(i2c, address, &reg, 1, true, 1000) == PICO_ERROR_GENERIC){
            return 0;
        }
        if (i2c_read_timeout_us(i2c, address, buffer, 2, false, 1000) == PICO_ERROR_GENERIC){
            return 0;
        }
        complete_buffer[2*i] = buffer[0];
        complete_buffer[2*i+1] = buffer[1];
        reg += 2;
    }

    int16_t rawX = (complete_buffer[1] << 8) | complete_buffer[0];
    int16_t rawY = (complete_buffer[3] << 8) | complete_buffer[2];
    int16_t rawZ = (complete_buffer[5] << 8) | complete_buffer[4];

    uint range = (int)(this->accelRange);
    float scale = accelRangeLookup[range];

    *x = ((float)rawX * scale / 32768.0) * 9.80665;
    *y = ((float)rawY * scale / 32768.0) * 9.80665;
    *z = ((float)rawZ * scale / 32768.0) * 9.80665;
    return 1;
    
}

void LSM6DSOX::setGyroOffsets(float x, float y, float z) {
    this->offsetGyroX = x;
    this->offsetGyroY = y;
    this->offsetGyroZ = z;
}

int LSM6DSOX::readGyro(float *x, float *y, float *z) {
    uint8_t buffer[2];
    uint8_t complete_buffer[6];
    uint8_t reg = LSM6DS_OUTX_L_G;

    // read out 2 values at a time, incrementing the register address
    for (uint i = 0; i < 3; i++) {
        if (i2c_write_timeout_us(i2c, address, &reg, 1, true, 1000) == PICO_ERROR_GENERIC) {
            return 0;
        }
        if (i2c_read_timeout_us(i2c, address, buffer, 2, false, 1000) == PICO_ERROR_GENERIC) {
            return 0;
        }
        complete_buffer[2 * i] = buffer[0];
        complete_buffer[2 * i + 1] = buffer[1];
        reg += 2;
    }

    int16_t rawX = (complete_buffer[1] << 8) | complete_buffer[0];
    int16_t rawY = (complete_buffer[3] << 8) | complete_buffer[2];
    int16_t rawZ = (complete_buffer[5] << 8) | complete_buffer[4];

    uint range = (int)(this->gyroRange);
    float scale = gyroRangeLookup[range];

    *x = ((float)rawX * scale / 32768.0) - this->offsetGyroX;
    *y = ((float)rawY * scale / 32768.0) - this->offsetGyroY;
    *z = ((float)rawZ * scale / 32768.0) - this->offsetGyroZ;

    return 1;
}

LSM6DSDataRate LSM6DSOX::getGyroDataRate() {
    uint8_t reg = LSM6DS_CTRL2_G;
    uint8_t buffer[1];
    i2c_write_blocking(i2c, address, &reg, 1, true);
    i2c_read_blocking(i2c, address, buffer, 1, false);
    this->gyroDataRate = (LSM6DSDataRate)(buffer[0] >> 4);
    return this->gyroDataRate;
}

LSM6DSGyroRange LSM6DSOX::getGyroRange() {
    uint8_t reg = LSM6DS_CTRL2_G;
    uint8_t buffer[1];
    i2c_write_blocking(i2c, address, &reg, 1, true);
    i2c_read_blocking(i2c, address, buffer, 1, false);
    this->gyroRange = (LSM6DSGyroRange)(buffer[0] & 0x0F); 
    return this->gyroRange;
}

LSM6DSDataRate LSM6DSOX::getAccelDataRate() {
    uint8_t reg = LSM6DS_CTRL1_XL;
    uint8_t buffer[1];
    i2c_write_blocking(i2c, address, &reg, 1, true);
    i2c_read_blocking(i2c, address, buffer, 1, false);
    this->accelDataRate = (LSM6DSDataRate)(buffer[0] >> 4); 
    return this->accelDataRate;
}

LSM6DSAccelRange LSM6DSOX::getAccelRange() {
    uint8_t reg = LSM6DS_CTRL1_XL;
    uint8_t buffer[1];
    i2c_write_blocking(i2c, address, &reg, 1, true);
    i2c_read_blocking(i2c, address, buffer, 1, false);
    this->accelRange = (LSM6DSAccelRange)((buffer[0] >> 2) & 0x03); 
    return this->accelRange;
}

bool LSM6DSOX::accelHasData() {
    uint8_t reg = LSM6DS_STATUS_REG;
    uint8_t buffer[1];
    i2c_write_blocking(i2c, address, &reg, 1, true);
    i2c_read_blocking(i2c, address, buffer, 1, false);
    return (buffer[0] & 0x01) != 0;
}

bool LSM6DSOX::gyroHasData() {
    uint8_t reg = LSM6DS_STATUS_REG;
    uint8_t buffer[1];
    i2c_write_blocking(i2c, address, &reg, 1, true);
    i2c_read_blocking(i2c, address, buffer, 1, false);
    return (buffer[0] & 0x02) != 0;
}

float LSM6DSOX::readTemp() {
    static const int tmp_lsb_deg = 256;
    static const int tmp_ofst_deg = 25;

    uint8_t buffer[2];
    uint8_t reg = LSM6DS_OUT_TEMP_L;
    
    float temp = 0.0;
    
    i2c_write_blocking(i2c, address, &reg, 1, true);
    int len = i2c_read_blocking(i2c, address, buffer, 2, false);
    
    if (len > 1) {
        int rawTemp = (buffer[1] << 8) | buffer[0];
        
        temp = (static_cast<float>(rawTemp) / tmp_lsb_deg) + tmp_ofst_deg;
    }
    else {
        temp = -273.15;
    }
    
    return temp;
}
