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
    _previousError = 0;
}

void PIDController::setSetpoint(int setpoint) 
{
    _setpoint = setpoint;
}

int PIDController::compute(int measurement, unsigned long timestamp)
{
    int output = 0;
    int error = _setpoint - measurement;

    float dT = (timestamp - _prevExecutionMillis) / 1000.0f;

    output += (int) (error * _pGain);

    _iTerm += error * _iGain * dT;
    _iTerm = constrain(_iTerm, _minIterm, _maxIterm);

    output += (int) _iTerm;

    _dTerm = (float)(error - _previousError) * _dGain;

    output += _dTerm;

    _previousError = error;
    _previousMeasurement = measurement;
    _prevExecutionMillis = timestamp;

    return constrain(output, _min, _max);
}

void PIDController::resetIterm() {
    _iTerm = 0;
}

float PIDController::getIterm() {
    return _iTerm;
}

float PIDController::getDterm() {
    return _dTerm;
}