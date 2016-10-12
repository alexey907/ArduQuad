#include "BaroAltHold.h"

BaroAltHold::BaroAltHold(IBaro* pBaro, MPUBase* pMPU){
  m_pBaro = pBaro;
  m_pMPU = pMPU;
  m_PID.setLimits(-75, 75);
  m_altVelo = 0;
  m_alt = 0;
}

int BaroAltHold::update(int throttle){

  if (!m_pBaro){
    return 0;
  }
  
  m_altVelo += UPDATE_INTERVAL * m_pMPU->getWorldAccel()->z * 9.8;
  
  //fuse data from accel and baro sensors
  m_altVelo += (m_pBaro->getVelocity() - m_altVelo) * 0.003; 
  
  
  m_alt += m_altVelo * UPDATE_INTERVAL;
  m_alt += (m_pBaro->getAltitude() - m_alt) * 0.001; 
  
  //Serial.print(m_pBaro->getAltitude());Serial.print(", ");Serial.print(m_alt); Serial.print(", "); Serial.println(m_altVelo);
  
  if (!isArmed(throttle)) {
    m_on = false;
    return 0;
  }
  //Serial.println("Baro alt hold armed");
  if (m_on == false) {
    m_altTarget = m_alt;
    m_on = true;
  }
  
  return  m_PID.compute(m_alt, m_altVelo, m_altTarget);
  
} 
bool BaroAltHold::isArmed(int throttle){
  
  if ((throttle < ALT_HOLD_MIN_THROTTLE) || (abs(throttle - m_lastThrottle) > ALT_HOLD_THRESHOLD)){
    //Serial.print(throttle);Serial.print(":"); Serial.println(m_lastThrottle);
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





