#include "HMC5883Compass.h"
#include "I2CDevice.h"
#include "vector.h"



bool  HMC5883Compass::begin(TwoWire* pWire) {
  m_pDevice = new I2CDevice(pWire, HMC5883L_ADDRESS);
  if (getID() != 0x4834){
    Serial.println("HMC5883, Compass not found!");
    while(true);
    
    return false;
  }

  setMode(HMC5883L_MODE_CONTINUOUS);
 // setConfig(HMC5883L_AVERAGING_8, HMC5883L_RATE_75, HMC5883L_BIAS_NORMAL);
 // setGain(HMC5883L_GAIN_1090);
  return true;
}

void HMC5883Compass::setMode(uint8_t mode) {
  m_pDevice->writeByte(HMC5883L_MODE, mode);
}

void HMC5883Compass::setGain(uint8_t gain) {
  m_pDevice->writeByte(HMC5883L_CONFIG_B, gain << 5);
}

void  HMC5883Compass::setConfig(uint8_t averaging, uint8_t rate, uint8_t bias) {
  m_pDevice->writeByte(HMC5883L_CONFIG_A, (averaging << 5) | (rate << 2) | bias );
}

bool HMC5883Compass::calibrate() {
  update();
  Serial.print(m_minX);Serial.print(" ... "); Serial.print(m_maxX); Serial.print(",");
  Serial.print(m_minY);Serial.print(" ... "); Serial.print(m_maxY); Serial.println();
  delay(20);
  return m_calCount-- < 0;
}

float HMC5883Compass::getHeading() {
  return m_heading;
}

uint16_t HMC5883Compass::getID(){
  return m_pDevice->readUInt(HMC5883L_IDA);
}

void HMC5883Compass::update() {
  
  if ((millis() - m_lastUpdate) < UPDATE_INTERVAL) {
    return;
  }
  m_lastUpdate = micros();
  
  byte data[6];
  
  m_pDevice->readBytes(HMC5883L_DATA, data, 6);
   
  int16_t x = data[0] << 8 | data[1];
  int16_t z = data[2] << 8 | data[3];
  int16_t y = data[4] << 8 | data[5];

  if (x > m_maxX) {
    m_maxX = x;
  }
  if (y > m_maxY) {
    m_maxY = y;
  }
  if (y < m_minY) {
    m_minY = y;
  }
  if (x < m_minX) {
    m_minX = x;
  }

  m_heading = atan2_.get((float) x - (m_maxX + m_minX) / 2,
                     (float) y - (m_maxY + m_minY) / 2);

}


