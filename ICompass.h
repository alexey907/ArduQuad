#ifndef __ICOMPASS_H__

#define __ICOMPASS_H__

#include <Wire.h>
class ICompass {
  public:
    virtual bool begin(TwoWire* pWire) = 0;
    virtual bool calibrate() = 0;
    virtual float getHeading() = 0;
    virtual void update();
};
#endif
