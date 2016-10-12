#include "Motor.h"
#include "ArduQuad.h"
#include <arduino.h>

bool Motor::m_armed = false;
uint16_t* Motor::m_throttleFn = NULL;

void Motor::init (){
  Serial.print("Throttle scale: ");Serial.println(THROTTLE_SCALE);
  m_throttleFn = new uint16_t[1001];
  for (int n = 0; n <= 1000; n++){
    m_throttleFn[n] = 1000 + pow(((float) n) / 1000.0, THROTTLE_SCALE) * 1000.0;  
  }
  m_armed = false;
}

void Motor::begin(int pin){
  m_esc.attach(pin);
  m_esc.writeMicroseconds(1000);
  
}


void Motor::setThrottle(int val){
  m_throttle = (m_armed) ? m_throttleFn[(val > 1000) ? 1000 : ((val < 0) ? 0 : val)] : 1000;
  m_esc.writeMicroseconds(m_throttle);
 
}

int Motor::getThrottle(){
  return m_throttle;
}

void Motor::setArmed(bool armed){
  m_armed = armed;
}


