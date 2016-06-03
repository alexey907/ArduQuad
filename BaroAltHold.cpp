#include "BaroAltHold.h"

BaroAltHold::BaroAltHold(AFBMP180* pBaro, MPUBase* pMPU){
  m_pBaro = pBaro;
  m_pMPU = pMPU;
  m_PID.setLimits(-100, 100);
  m_altVelo = 0;
}

int BaroAltHold::update(int throttle){
  
  m_altVelo += UPDATE_INTERVAL * (m_pMPU->getAccel()->z - 1.0) * 9.8;
  
  //fuse data from accel and baro sensors
  m_altVelo += (m_pBaro->getVelocity() - m_altVelo) * 0.003; 
  //Serial.print(m_pBaro->getVelocity()); Serial.print(", "); Serial.println(m_altVelo);
  
  if (!isArmed(throttle)) {
    m_on = false;
    return 0;
  }

  if (m_on == false) {
    m_altTarget = m_pBaro->getAltitude();
    m_on = true;
  }
  
  return  m_PID.compute(m_pBaro->getAltitude(), m_altVelo, m_altTarget);
  
} 
bool BaroAltHold::isArmed(int throttle){
  
  if ((throttle < ALT_HOLD_MIN_THROTTLE) || (abs(throttle - m_lastThrottle) > ALT_HOLD_THRESHOLD)){
    m_lastThrottle = throttle;
    m_holdCount = 0;
    return false;
  }
  
  m_holdCount++; //count how many ticks throttle stays unchanged within threshold
  
  if (m_holdCount < ALT_HOLD_CYCLES){
    return false;
  } 
  
  return true;
}

bool BaroAltHold::isOn(){
  return m_on;
}





