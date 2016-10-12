#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <Servo.h>

class Motor {
  public:
    static void init();
    static void setArmed(bool armed);
    
    void begin(int pin);
    void setThrottle(int throttle);
    int getThrottle();
    
  private:
    
    Servo m_esc;
    int m_throttle;
    static uint16_t* m_throttleFn;
    static bool m_armed;
      
};
#endif
