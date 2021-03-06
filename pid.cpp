#include "pid.h"
#include <Arduino.h>

PIDController::PIDController(float pGain, float iGain, float dGain, float ffGain)
{
    _pGain = pGain;
    _iGain = iGain;
    _dGain = dGain;
    _ffGain = ffGain;

    _minIterm = -250;
    _maxIterm = 250;

    _min = 0;
    _max = 255;
    _previousError = 0;
}

void PIDController::setProperties(int minOutput, int maxOutput)
{
    _min = minOutput;
    _max = maxOutput;
}

void PIDController::setItermProperties(int minIterm, int maxIterm) 
{
    _minIterm = minIterm;
    _maxIterm = maxIterm;
}

void PIDController::setSetpoint(int setpoint) 
{
    _setpoint = setpoint;
}

int PIDController::compute(int measurement, unsigned long timestamp)
{
    int output = 0;
    int error = _setpoint - measurement;

    //Do not run update if pid loop is called too often
    if (timestamp - _prevExecutionMillis < 1000) {
        float dT = (timestamp - _prevExecutionMillis) / 1000.0f;

        //pTerm
        output += (int) (error * _pGain);

        //Apply and constrain iTerm
        _iTerm += error * _iGain * dT;
        _iTerm = constrain(_iTerm, _minIterm, _maxIterm);
        output += (int) _iTerm;

        //dTerm
        _dTerm = (float)(error - _previousError) * _dGain * dT;
        output += _dTerm;

        //ffTerm
        _ffTerm = (float) _setpoint * _ffGain;
        output += _ffTerm;
    }

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