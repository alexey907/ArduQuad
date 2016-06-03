#ifndef __DUEPWM_H__
#define __DUEPWM_H__
#define SIGNAL_TIMEOUT 1000000
#define RANGE_CHECK 50

#define CREATE_PWM(name, pin) void myHandler_##name(); \
  DuePWM name(pin, myHandler_##name);\
  void myHandler_##name() {name.onChange();}


class DuePWM {
  public:
    DuePWM(int _pin, void (*handler)());

    int getValue();
    void onChange();
    void setZero(int zero);
    bool isReady();
  private:
    int pin;
    long pinTimer;
    int pinValue;
    long lastChange;
    int zeroPoint;

    bool maxCheck = false;
    bool ready = false;

};

#endif
