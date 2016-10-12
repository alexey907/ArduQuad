#ifndef __BATTERY_H__
#define __BATTERY_H__
#include <Arduino.h>
#include "LPF.h"

#define LIPO1S_MIN 3200
#define LIPO1S_MAX 4300
#define USB_MAX 5000


class Battery {
  public:

    Battery();
    int getCells();
    int getMillivolts();
    int getUsed();
    int getRemaining();
   
    

  private:
    int m_nCells = -1; //0 means device is USB powered
    LPF m_lpfVoltage;
};
#endif
