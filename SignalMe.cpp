#include "SignalMe.h"


SignalMe::SignalMe(int _pin) {
	pin = _pin;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
}

void SignalMe::play(int high, int low, int count){
	
	for (int n = 0; n < count; n++){
		set(HIGH);
		delay(high);
		set(LOW);
		delay(low);
	}
	
    
}

void SignalMe::cycle(int high, int low){
	
	set ((millis() % (high + low) < high) ? HIGH : LOW);
	
}
void SignalMe::set(int state){
	digitalWrite(pin, state);
}

