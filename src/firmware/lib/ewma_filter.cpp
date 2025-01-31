// based on https://hackaday.com/2019/09/06/sensor-filters-for-coders/
#include "ewma_filter.hpp"

EWMAFilter::EWMAFilter(float decay, float initialValue)
    : decayFactor(decay), previousFilteredValue(initialValue) {
}

void EWMAFilter::addData(float data) {
    previousFilteredValue = (1 - decayFactor) * previousFilteredValue + decayFactor * data;
}

float EWMAFilter::getFilteredData() const {
    return previousFilteredValue;
}
