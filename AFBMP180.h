
#ifndef __AFBMP180_H__
#define __AFBMP180_H__

#include <Arduino.h>

#include "ArduQuad.h"
#include "I2CDevice.h"
#include "LPF.h"


#define BMP180_ADDR 0x77 // 7-bit address

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4
#define CYCLE_DURATION 25000

#define CYCLE_TICKS (CYCLE_DURATION / TIME_STEP)
class AFBMP180
{
  public:
    void begin(TwoWire* pWire);
    void updateReading();
    bool calibrate();
    float getAltitude();
    float getVelocity();



  private:

    void startTemperature();

    void readTemperature();

    void startPressure();

    void readPressure();

    I2CDevice* m_pDevice;
    LPF m_lpfPressure;

    int32_t m_B5;

    double m_A0;

    int m_tick = 0;
    struct {
      int16_t ac1, mb;
      int32_t ac2 , ac3, b1, b2, mc, md;
      uint32_t ac4;
      int32_t ac5, ac6;
    } m_cal;

    float m_alt = 0;
    float m_lastAlt = 0;
    float m_velocity = 0;
};


#endif
