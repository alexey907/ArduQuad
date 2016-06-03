#include "Arduino.h"
#include "Logger.h"

Logger::Logger(Print* p1, Print* p2)
{
	this->p1 = p1;
	this->p2 = p2;
}

size_t Logger::write(uint8_t byte) {
	p1->write(byte);
	p2->write(byte);	
}
