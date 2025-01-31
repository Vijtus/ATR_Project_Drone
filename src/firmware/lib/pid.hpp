#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float cycleTime, float iLimit = 10.0);
    float calculate(float actual, float goal);
    void reset();
    void updateConstants(float kp, float ki, float kd);

private:
    float kp;
    float ki;
    float kd;
    float cycleTime;
    float iLimit;
    float lastError;
    float lastI;
};

#endif // PIDCONTROLLER_HPP
