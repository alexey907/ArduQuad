/*
    AFBMP180.cpp
    Bosch AFBMP180 pressure sensor library for the Arduino microcontroller
    Mike Grusin, SparkFun Electronics

    Uses floating-point equations from the Weather Station Data Logger project
    http://wmrx00.sourceforge.net/
    http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

    Forked from BMP085 library by M.Grusin

    version 1.0 2013/09/20 initial version
    Verison 1.1.2 - Updated for Arduino 1.6.4 5/2015

    Our example code uses the "beerware" license. You can do anything
    you like with this code. No really, anything. If you find it useful,
    buy me a (root) beer someday.
*/

#include "AFBMP180.h"
#include "I2CDevice.h"
#include <stdio.h>
#include <math.h>





void AFBMP180::begin(TwoWire* pWire) {

  m_pDevice = new I2CDevice(pWire, BMP180_ADDR);
  m_lpfPressure.setFeedback(0.5);
  
  m_cal.ac1 = m_pDevice->readInt(0xAA);
  m_cal.ac2 = m_pDevice->readInt(0xAC);
  m_cal.ac3 = m_pDevice->readInt(0xAE);
  m_cal.ac4 = m_pDevice->readUInt(0xB0);
  m_cal.ac5 = m_pDevice->readInt(0xB2);
  m_cal.ac6 = m_pDevice->readInt(0xB4);
  m_cal.b1 = m_pDevice->readInt(0xB6);
  m_cal.b2 = m_pDevice->readInt(0xB8);
  m_cal.mb = m_pDevice->readInt(0xBA);
  m_cal.mc = m_pDevice->readInt(0xBC);
  m_cal.md = m_pDevice->readInt(0xBE);


}
bool AFBMP180::calibrate() {

  updateReading();

  delay(5);
  if (m_tick > CYCLE_TICKS * 10) {
     m_A0 = getAltitude();
     m_lpfPressure.setFeedback(BARO_LPF);
    return true;
  }
  return false;
}
void AFBMP180::updateReading() {
  switch ((m_tick % CYCLE_TICKS) * TIME_STEP) {
    case 0: //0ms
      startTemperature();
      break;
    case 5000: //5ms
      readTemperature();
      break;
    case 10000: //10ms
      startPressure();
      break;
    case 15000: //20ms
      readPressure();
      break;
  }
  
  m_tick++;
}

void AFBMP180::startTemperature() {
  m_pDevice->writeByte(BMP180_REG_CONTROL, BMP180_COMMAND_TEMPERATURE);
}

void AFBMP180::readTemperature() {
  int32_t ut = m_pDevice->readInt(BMP180_REG_RESULT);
  int32_t X1 = (ut - m_cal.ac6) * m_cal.ac5 >> 15;
  int32_t X2 = (m_cal.mc << 11) / (X1 + m_cal.md);
  m_B5 = X1 + X2;
}


void AFBMP180::startPressure() {
  m_pDevice->writeByte(BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE0);
}


void AFBMP180::readPressure() {
  
  
  int32_t up = m_pDevice->readUInt(BMP180_REG_RESULT);

  up <<= 8;
  up |= m_pDevice->readByte(BMP180_REG_RESULT + 2);
  up >>= 8;
  int32_t B6 = m_B5 - 4000;
  
  int32_t X1, X2, X3;
  
  X3 = ((m_cal.b2 * ((B6 * B6) >> 12)) >> 11) + ((m_cal.ac2 * B6) >> 11);
  int32_t B3 = (m_cal.ac1 * 4 + X3 + 2) / 4;
 
 
  X1 = (m_cal.ac3 * B6) >> 13;
  X2 = (m_cal.b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = (m_cal.ac4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t) up - B3) * 50000UL;
 
  int32_t p;

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  
  X1 = ((p >> 8) * (p >> 8) * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  p = p + ((X1 + X2 + 3791) >> 4);
  
  m_lpfPressure.filter(p);
  m_alt = (44330.0 * (1 - pow(m_lpfPressure.get() / 101500, 0.1903))) - m_A0;
  m_velocity = (m_alt - m_lastAlt) / (CYCLE_DURATION / 1000000.0);
  m_lastAlt = m_alt;
}


float AFBMP180::getAltitude() {
  return m_alt;
}

float AFBMP180::getVelocity() {
  return m_velocity;
}



