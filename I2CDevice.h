#ifndef __I2CDEVICE_H__
#define __I2CDEVICE_H__

#include <Wire.h>


class I2CDevice{
  public:
    I2CDevice (TwoWire* pWire, uint8_t addr);
    uint8_t readByte(uint8_t reg);
    int16_t readInt(uint8_t reg);
    uint16_t readUInt(uint8_t reg);
    void readBytes(uint8_t reg, uint8_t* data, int size);
    void writeByte(uint8_t reg, uint8_t val);
    void writeReg(uint8_t reg);


  
  private:
    TwoWire* m_pWire;
    uint8_t m_addr; 
};

#endif
