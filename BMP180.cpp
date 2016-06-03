/*
	BMP180.cpp
	Bosch BMP180 pressure sensor library for the Arduino microcontroller
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

#include "BMP180.h"
#include "I2CDevice.h"
#include <stdio.h>
#include <math.h>





void BMP180::begin(TwoWire* pWire) {

  m_pDevice = new I2CDevice(pWire, BMP180_ADDR);
  m_lpfAlt.setFeedback(0.5);
  double c3, c4, b1;


  int16_t AC1 = m_pDevice->readInt(0xAA);
  int16_t AC2 = m_pDevice->readInt(0xAC);
  int16_t AC3 = m_pDevice->readInt(0xAE);
  uint16_t AC4 = m_pDevice->readInt(0xB0);
  uint16_t AC5 = m_pDevice->readInt(0xB2);
  uint16_t AC6 = m_pDevice->readInt(0xB4);
  int16_t VB1 = m_pDevice->readInt(0xB6);
  int16_t VB2 = m_pDevice->readInt(0xB8);
  int16_t MB = m_pDevice->readInt(0xBA);
  int16_t MC = m_pDevice->readInt(0xBC);
  int16_t MD = m_pDevice->readInt(0xBE);


  // All reads completed successfully!

  // If you need to check your math using known numbers,
  // you can uncomment one of these examples.
  // (The correct results are commented in the below functions.)

  // Example from Bosch datasheet
  // AC1 = 408; AC2 = -72; AC3 = -14383; AC4 = 32741; AC5 = 32757; AC6 = 23153;
  // B1 = 6190; B2 = 4; MB = -32768; MC = -8711; MD = 2868;

  // Example from http://wmrx00.sourceforge.net/Arduino/BMP180-Calcs.pdf
  // AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
  // VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;


  c3 = 160.0 * pow(2, -15) * AC3;
  c4 = pow(10, -3) * pow(2, -15) * AC4;
  b1 = pow(160, 2) * pow(2, -30) * VB1;
  m_cal.c5 = (pow(2, -15) / 160) * AC5;
  m_cal.c6 = AC6;
  m_cal.mc = (pow(2, 11) / pow(160, 2)) * MC;
  m_cal.md = MD / 160.0;
  m_cal.x0 = AC1;
  m_cal.x1 = 160.0 * pow(2, -13) * AC2;
  m_cal.x2 = pow(160, 2) * pow(2, -25) * VB2;
  m_cal.y0 = c4 * pow(2, 15);
  m_cal.y1 = c4 *  c3;
  m_cal.y2 = c4 * b1;
  m_cal.p0 = (3791.0 - 8.0) / 1600.0;
  m_cal.p1 = 1.0 - 7357.0 * pow(2, -20);
  m_cal.p2 = 3038.0 * 100.0 * pow(2, -36);

}
bool BMP180::calibrate() {

  updateReading();

  delay(10);
  if (m_tick > CYCLE_DURATION * 10) {
    m_P0 = m_lpfAlt.get();
     m_lpfAlt.setFeedback(0.1);
    return true;
  }
  return false;
}
void BMP180::updateReading() {
  switch ((m_tick % CYCLE_DURATION) * TIME_STEP) {
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

void BMP180::startTemperature() {
  m_pDevice->writeByte(BMP180_REG_CONTROL, BMP180_COMMAND_TEMPERATURE);
}

void BMP180::readTemperature() {

  double tu = m_pDevice->readInt(BMP180_REG_RESULT);


  //example from Bosch datasheet
  //tu = 27898;

  //example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
  //tu = 0x69EC;

  double a = m_cal.c5 * (tu - m_cal.c6);
  m_T = a + (m_cal.mc / (a + m_cal.md));

  


}


void BMP180::startPressure() {
  m_pDevice->writeByte(BMP180_REG_CONTROL, BMP180_COMMAND_PRESSURE0);
}


void BMP180::readPressure() {
  unsigned char data[3];
  m_pDevice->readBytes(BMP180_REG_RESULT, data, 3);

  double pu = (data[0] * 256.0) + data[1] + (data[2] / 256.0);
  //Serial.print("pu=");Serial.println(pu);

  //example from Bosch datasheet
  //pu = 23843;

  //example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;
  //pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);

  double s = m_T - 25.0;

  double z = (pu - (m_cal.x2 * s * s) + (m_cal.x1 * s) + m_cal.x0) /
             ((m_cal.y2 * s * s) + (m_cal.y1 * s) + m_cal.y0);

  //Serial.println(m_T);
  //Serial.println((m_cal.p2 * z * z) + (m_cal.p1 * z) + m_cal.p0);
  
  m_lpfAlt.filter((m_cal.p2 * z * z) + (m_cal.p1 * z) + m_cal.p0);
  

}


double BMP180::getAltitude() {
 
  return (44330.0 * (1 - pow(m_lpfAlt.get() / m_P0, 1 / 5.255)));
}



