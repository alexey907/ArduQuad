#ifndef __PID_h__
#define __PID_h__

#include "avg.h"



class PID
{


  public:

  PID(double Kp, double Kd, double Ki);     
	double compute(double input, double target); 
	double compute(double input, double dErr, double target); 
  void setLimits(double min, double max); 
	void reset();

   private:
   
	double Kp;                  // * (P)roportional Tuning Parameter
  double Ki;                  // * (I)ntegral Tuning Parameter
  double Kd;                  // * (D)erivative Tuning Parameter

	double lastInput;
	
	double outMin;
	double outMax;
	
	Avg* pAvgErr = new Avg(100);
	


};
#endif

