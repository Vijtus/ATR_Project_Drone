#ifndef EWMAFILTER_HPP
#define EWMAFILTER_HPP

class EWMAFilter {
public:
    EWMAFilter(float decayFactor, float initialValue);
    void addData(float data);
    float getFilteredData() const;

private:
    float decayFactor;
    float previousFilteredValue;
};

#endif // EWMAFILTER_HPP
