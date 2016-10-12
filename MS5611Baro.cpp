#include "MS5611baro.h"

#include "I2CDevice.h"

//Hardware conf:
//PS:low
//CSB:high


void MS5611Baro::begin(TwoWire* pWire) {

  m_pDevice = new I2CDevice(pWire, MS5611_ADDR);
  m_lpfPressure.setFeedback(0.5);

  m_pDevice->writeReg(0x1E);

  delay(500);
  m_cal.C1 = m_pDevice->readUInt(0xA2);
  m_cal.C2 = m_pDevice->readUInt(0xA4);
  m_cal.C3 = m_pDevice->readUInt(0xA6);
  m_cal.C4 = m_pDevice->readUInt(0xA8);
  m_cal.C5 = m_pDevice->readUInt(0xAA);
  m_cal.C6 = m_pDevice->readUInt(0xAC);
  
  
  Serial.println(m_cal.C1);
  Serial.println(m_cal.C2);
  Serial.println(m_cal.C3);
  Serial.println(m_cal.C4);
  Serial.println(m_cal.C5);
  Serial.println(m_cal.C6);


  m_cal.C1 = m_cal.C1 << 15;
  m_cal.C2 = m_cal.C2 << 16;
  m_cal.C5 = m_cal.C5 << 8;
  m_cal.C6 = m_cal.C6 >> 1;
}
bool MS5611Baro::calibrate() {

  updateReading();

  delay(3);
  if (m_tick > CYCLE_TICKS * 100) {
    m_A0 = getAltitude();
    m_lpfPressure.setFeedback(BARO_LPF);
    return true;
  }
  return false;
}
void MS5611Baro::updateReading() {
  
  switch ((m_tick % CYCLE_TICKS) * TIME_STEP) {
    case 0: //0ms
      startPressure();
      break;
    case 10000: //5ms
      readPressure();
      startTemperature();
      break;
    case 20000: //20ms
      readTemperature();
      break;
  }

  m_tick++;
}

void MS5611Baro::startTemperature() {
  Serial.print(millis()); Serial.println(" startTemperature");
  m_pDevice->writeReg(MS5611_TEMPERATURE);
}

void MS5611Baro::readTemperature() {
  Serial.print(millis()); Serial.println(" readTemperature");
  
  m_pDevice->readBytes(MS5611_RAWADC, m_rawData, 3);
  unsigned long rawTemp = m_rawData[0] << 16 | m_rawData[1] << 8 | m_rawData[0];
  unsigned long dT = rawTemp - m_cal.C5;

  unsigned long temp = 2000 + ((dT * m_cal.C6) >> 22);

  // Offset and Sensitivity calculation
  unsigned long long off = m_cal.C2  + ((m_cal.C4 * dT) >> 7);
  unsigned long long sens = m_cal.C1 + ((m_cal.C3 * dT) >> 8);

  // 2nd order temperature and pressure compensation
  if (temp < 2000) {
    temp -= (dT * dT) >> 31;
    long temp2 = (rawTemp - 2000) * (rawTemp - 2000) * 5;
    off -= temp2 >> 1;
    sens -=  temp2 >> 2;
    if (temp < -1500)
    {
      temp2 = (rawTemp + 1500) * (rawTemp + 1500);
      off  -= 7 * temp2;
      sens -= 11 * temp2 >> 1;
    }
  }


  // Convert the final data
  long pressure = (m_rawPressure * sens / 2097152 - off) >> 15;
  
  m_lpfPressure.filter(pressure);
  m_alt = (44330.0 * (1 - pow(m_lpfPressure.get() / 101500, 0.1903))) - m_A0;
  m_velocity = (m_alt - m_lastAlt) / (CYCLE_DURATION / 1000000.0);
  m_lastAlt = m_alt;
}


void MS5611Baro::startPressure() {
   Serial.print(millis()); Serial.println(" startPressure");
 
  m_pDevice->writeReg(MS5611_PRESSURE);
}


void MS5611Baro::readPressure() {
   Serial.print(millis()); Serial.println(" readPressure");
 
  m_pDevice->readBytes(MS5611_RAWADC, m_rawData, 3);
  m_rawPressure = m_rawData[0] << 16 | m_rawData[1] << 8 | m_rawData[0];

}


float MS5611Baro::getAltitude() {
  return m_alt;
}

float MS5611Baro::getVelocity() {
  return m_velocity;
}



