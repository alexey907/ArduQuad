#include "I2CDevice.h"

I2CDevice::I2CDevice(TwoWire* pWire, uint8_t addr) {
  m_pWire = pWire;
  m_addr = addr;


}
void I2CDevice::writeByte(uint8_t reg, uint8_t val) {
 /* Serial.print(">>>");
  Serial.print(reg);
  Serial.print(":");
  Serial.println(val);
  */
  m_pWire->beginTransmission(m_addr);
  m_pWire->write(reg);
  m_pWire->write(val);
  m_pWire->endTransmission();
}





uint8_t I2CDevice::readByte(uint8_t reg) {
  m_pWire->beginTransmission(m_addr);
  m_pWire->write(reg);
  m_pWire->endTransmission(false);
  m_pWire->requestFrom(m_addr, 1, true);
  return m_pWire->read();
  /*Serial.print("<<<");
  Serial.print(reg);
  Serial.print(":");
  Serial.println(val);
  return val;*/
}

int16_t I2CDevice::readInt(uint8_t reg) {
  m_pWire->beginTransmission(m_addr);
  m_pWire->write(reg);
  m_pWire->endTransmission(false);
  m_pWire->requestFrom(m_addr, 2, true);
  return m_pWire->read() << 8 | m_pWire->read();
}

uint16_t I2CDevice::readUInt(uint8_t reg) {
  m_pWire->beginTransmission(m_addr);
  m_pWire->write(reg);
  m_pWire->endTransmission(false);
  m_pWire->requestFrom(m_addr, 2, true);
  return ((uint16_t) m_pWire->read() << 8) | (uint16_t) m_pWire->read();
}

void I2CDevice::readBytes(uint8_t reg, uint8_t* data, int size) {
  Wire.beginTransmission(m_addr);
  Wire.write(reg);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(m_addr, size, true); // request a total of 14 registers
  for (int n = 0; n < size; n++) {
    data[n] = m_pWire->read();
  }


}

