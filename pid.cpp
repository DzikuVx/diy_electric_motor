#include "pid.h"
#include <Arduino.h>

PIDController::PIDController(float pGain, float iGain, float dGain)
{
    _pGain = pGain;
    _iGain = iGain;
    _dGain = dGain;

    _minIterm = -150;
    _maxIterm = 150;

    _min = 0;
    _max = 255;
}

void PIDController::setSetpoint(int setpoint) 
{
    _setpoint = setpoint;
}

int PIDController::compute(int measurement)
{
    int output = 0;
    int error = _setpoint - measurement;

    output += (int) (error * _pGain);

    _iTerm += error * _iGain;
    _iTerm = constrain(_iTerm, _minIterm, _maxIterm);

    output += (int) _iTerm;

    return constrain(output, _min, _max);
}

void PIDController::resetIterm() {
    _iTerm = 0;
}

float PIDController::getIterm() {
    return _iTerm;
}