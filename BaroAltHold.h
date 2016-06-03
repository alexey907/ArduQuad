#ifndef __BAROALTHOLD_H__
#define __BAROALTHOLD_H__

#include "PID.h"
#include "AFBMP180.h"
#include "MPUBase.h"
#include "ArduQuad.h"

#define ALT_HOLD_THRESHOLD 10
#define ALT_HOLD_CYCLES 100
#define ALT_HOLD_MIN_THROTTLE 1200

class BaroAltHold{
  public:
    BaroAltHold(AFBMP180* pBaro, MPUBase* pMPU);
    int update (int throttle);
    bool isOn();
    int getValue();
  private:

    boolean isArmed(int throttle);
    
    AFBMP180* m_pBaro;
    MPUBase* m_pMPU; 
   
    PID m_PID = PID(ALT_KP, ALT_KD, ALT_KI);
    
    int m_lastThrottle = 0;
    int m_holdCount = 0;
    bool m_on = false;
    int m_value = 0;
    
    float m_altVelo = 0;
    float m_altTarget = 0;
    
    
    
};
#endif
