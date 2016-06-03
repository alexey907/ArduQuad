
#ifndef __BMP180_H__
#define __BMP180_H__

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

#define CYCLE_DURATION (25000 / TIME_STEP)
class BMP180
{
	public:
	  void begin(TwoWire* pWire);
    void updateReading();
    bool calibrate();

		double getAltitude();

	
	private:

    void startTemperature();
    
    void readTemperature();

    void startPressure();

    void readPressure();
    
	  I2CDevice* m_pDevice;
    LPF m_lpfAlt;
    double m_P0;
    double m_T;
    int m_tick = 0;
	  struct {
		  double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
    } m_cal;
		
};


#endif
