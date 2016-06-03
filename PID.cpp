#include "Arduino.h"
#include "PID.h"

PID::PID(double _Kp, double _Kd, double _Ki)
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	outMin = -1000;
	outMax = 1000;
	
}
 
double PID::compute(double input, double target)
{
	
    
    
	double dErr = (input - lastInput);	
    lastInput = input;
    
    return compute(input, dErr, target);
}

double PID::compute(double input, double dErr, double target)
{
	  double pErr = target - input;
    
    pAvgErr->add(pErr);
    
    double iErr = pAvgErr->get();
 
    double output = Kp * pErr - Kd * dErr + Ki * iErr;
      
	output = (output > outMax) ? outMax : output;
	output = (output < outMin) ? outMin : output;
	
    
    return output;
}

void PID::setLimits(double Min, double Max)
{
   outMin = Min;
   outMax = Max;
}
 
void PID::reset()
{
   pAvgErr->reset();
   lastInput = 0;
}

