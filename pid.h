#ifndef PPMReader_h

#define PPMReader_h

class PIDController
{
  public:
    PIDController(float pGain, float iGain, float dGain);
    int compute(int measurement);
    void setSetpoint(int setpoint);
    void resetIterm();
    float getIterm();

  private:
    float _pGain;
    float _iGain;
    float _dGain;
    float _iTerm;
    int _error;
    int _minIterm;
    int _maxIterm;
    int _setpoint;
    int _min;
    int _max;
};

#endif