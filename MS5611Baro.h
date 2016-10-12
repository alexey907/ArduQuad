
#ifndef __MS5611BARO_H__
#define __MS5611BARO_H__

#include <Arduino.h>

#include "ArduQuad.h"
#include "I2CDevice.h"
#include "LPF.h"
#include "Baro.h"

#define MS5611_ADDR 0x77



#define MS5611_RAWADC       0x00
#define MS5611_RESET        0x1E
#define MS5611_PRESSURE     0x48
#define MS5611_TEMPERATURE  0x58
#define MS5611_CALIBRATION  0xA2



#define CYCLE_DURATION 25000

#define CYCLE_TICKS (CYCLE_DURATION / TIME_STEP)
class MS5611Baro : public IBaro {
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

    int m_tick = 0;
    struct {
      unsigned long C1, C2, C3, C4, C5, C6;
    } m_cal;

    uint8_t m_rawData[3];
    uint32_t m_rawPressure;

    float m_alt = 0;
    float m_A0 = 0;
    
    float m_lastAlt = 0;
    float m_velocity = 0;
};


#endif
