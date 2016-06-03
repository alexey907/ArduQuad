#ifndef __SIGNALME_H__
#define __SIGNALME_H__
#include "Arduino.h"

#define BLUE_LED 38
#define GREEN_LED 40
#define BUZZER 46



class SignalMe{
	
    public:
    	SignalMe(int pin);
        void play(int high, int low, int count);
        void cycle(int low, int high);
        void set(int state);
  	private: 
  		int pin;      
};

#endif