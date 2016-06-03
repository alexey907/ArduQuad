#include "DuePWM.h"
#include <Arduino.h>


                                                              
DuePWM::DuePWM(int _pin, void (*handler)()){
	pin = _pin;
	pinMode(pin, INPUT);
	attachInterrupt(pin, handler, CHANGE);
	
	zeroPoint = 1500;
	lastChange = 0;
	pinTimer = 0;
}

void DuePWM::onChange(){
	
	lastChange = micros();
	if (digitalRead(pin)) {
		pinTimer = lastChange;
	} else {
		pinValue = lastChange - pinTimer;
		pinTimer = 0;
	}
}

int DuePWM::getValue(){
	return ((micros() - lastChange) <  SIGNAL_TIMEOUT) ? pinValue : zeroPoint;
}

void DuePWM::setZero(int _zeroPoint){
	zeroPoint = _zeroPoint;	
}

bool DuePWM::isReady(){
	if ((micros() - lastChange) >  SIGNAL_TIMEOUT) 
		return false;
	
	if (abs(pinValue - 2000) < RANGE_CHECK){
    maxCheck = true;
  }
	if (abs(pinValue - 1000) < RANGE_CHECK){
		ready = maxCheck;
	}
	

	return ready;
		
} 
