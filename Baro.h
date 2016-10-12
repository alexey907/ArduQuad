#ifndef __BARO_H__
#define __BARO_H__
class IBaro{
  public:
    virtual void begin(TwoWire* pWire) = 0;
    virtual void updateReading() = 0;
    virtual bool calibrate() = 0;
    virtual float getAltitude() = 0;
    virtual float getVelocity() = 0;
};
#endif
