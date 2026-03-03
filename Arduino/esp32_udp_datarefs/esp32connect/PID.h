// PID.h
//#define PID_DEBUG // descomente se quiser ver debug

#include <Arduino.h>
class PID {
public:

    PID(float Kp_, float Ki_, float Kd_, float Kv_, float outMin_, float outMax_,char id_);

    float update(float setpoint, float measurement,float vel, float dt);
    void setTunings(float Kp_, float Ki_, float Kd_, float Kv_);
    void setOutputLimits(float minOut, float maxOut);
    void setIntegratorLimits(float minI, float maxI);
    void reset();

private:
    float Kp, Ki, Kd, Kv;
    float outMin, outMax;
    float iMin, iMax;

    float integrator;
    float prevError;
    char id; 
};
