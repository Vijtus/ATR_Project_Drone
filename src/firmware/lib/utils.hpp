#pragma once

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

float normalize(float value, float min, float max, float range_min, float range_max){
    float normalized = (value - min) / (max - min);
    float ret = normalized * (range_max - range_min) + range_min;
    // make sure to clamp in case the value is out of range
    if(ret > range_max){
        ret = range_max;
    }else if(ret < range_min){
        ret = range_min;
    }
    return ret;
}

float map(float value, float min, float max, float range_min, float range_max){
    float ret = (value - min) / (max - min) * (range_max - range_min) + range_min;
    // make sure to clamp in case the value is out of range
    if(ret > range_max){
        ret = range_max;
    }else if(ret < range_min){
        ret = range_min;
    }
    return ret;
}

float clamp(float value, float min, float max){
    if(value > max){
        return max;
    }else if(value < min){
        return min;
    }else{
        return value;
    }
}

uint clamp(uint value, uint min, uint max){
    if(value > max){
        return max;
    }else if(value < min){
        return min;
    }else{
        return value;
    }
}
