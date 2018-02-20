#ifndef PPMReader_h

#define PPMReader_h

class PIDController
{
  public:
    PIDController(float pGain, float iGain, float dGain);
    int compute(int measurement, unsigned long timestamp);
    void setSetpoint(int setpoint);
    void resetIterm();
    float getIterm();
    float getDterm();

  private:
    float _pGain;
    float _iGain;
    float _dGain;
    float _iTerm;
    float _dTerm;
    int _error;
    int _minIterm;
    int _maxIterm;
    int _setpoint;
    int _min;
    int _max;
    int _previousError;
    int _previousMeasurement;
    unsigned long _prevExecutionMillis;
};

#endif