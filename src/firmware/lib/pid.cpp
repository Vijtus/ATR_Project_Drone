// based on https://pidexplained.com/pid-controller-explained/
// and https://www.mathworks.com/videos/series/understanding-pid-control.html
#include "pid.hpp"

PIDController::PIDController(float kp, float ki, float kd, float cycleTime, float iLimit)
    : kp(kp), ki(ki), kd(kd), cycleTime(cycleTime), iLimit(iLimit),
      lastError(0.0f), lastI(0.0f)
{
    
}

// this is useful if we want to do real time tuning of the PID constants
void PIDController::updateConstants(float kp, float ki, float kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PIDController::calculate(float actual, float goal)
{
    float error = goal - actual;

    float P = error * kp;

    float I = lastI + (error * ki * cycleTime);
    // Simple clamping to a maximum value to prevent windup
    // https://www.mathworks.com/videos/understanding-pid-control-part-2-expanding-beyond-a-simple-integral-1528310418260.html
    if (I > iLimit) I = iLimit;
    if (I < -iLimit) I = -iLimit;

    // We will assume that the EWMA filter has removed enough noise that we don't need to do any filtering here
    float D = kd * (error - lastError) / cycleTime;

    lastError = error;
    lastI = I;

    float ret = P + I + D;
    return ret;
}

void PIDController::reset()
{
    lastError = 0.0f;
    lastI = 0.0f;
}
