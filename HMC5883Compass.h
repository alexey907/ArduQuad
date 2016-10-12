#ifndef __HMC5883Compass_H__
#define __HMC5883Compass_H__


#include <Arduino.h>

#include "I2CDevice.h"
#include "ICompass.h"

#define HMC5883L_ADDRESS 0x1E //0011110b, I2C 7bit address of HMC5883

#define HMC5883L_CONFIG_A        0x00
#define HMC5883L_CONFIG_B        0x01
#define HMC5883L_MODE            0x02
#define HMC5883L_DATA            0x03
#define HMC5883L_STATUS          0x09
#define HMC5883L_IDA             0x0A
#define HMC5883L_IDB             0x0B
#define HMC5883L_IDC             0x0C

 

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define UPDATE_INTERVAL 15 

class HMC5883Compass : public ICompass {
  public:
    bool begin(TwoWire* pWire);
    bool calibrate();
    float getHeading();
    void update();
    void setMode(uint8_t mode);
    void setGain(uint8_t gain);
    void setConfig(uint8_t averaging, uint8_t rate, uint8_t bias);
    uint16_t getID();
    
    
  private:
     I2CDevice* m_pDevice;
     int16_t m_minX = -362;
     int16_t m_minY = -413;
     int16_t m_maxX = 336;
     int16_t m_maxY = 253;
     long m_calCount = 0; 
     
     float m_heading = 0;
     
     long  m_lastUpdate = millis();

   
};
#endif
