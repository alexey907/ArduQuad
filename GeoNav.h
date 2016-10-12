#ifndef __GEONAV_H__
#define __GEONAV_H__

class IGeoNav {
public:
  virtual long getLon() = 0;
  virtual long getLat() = 0;
  virtual long getSignal() = 0;
  virtual bool init() = 0;
  virtual void updateData() = 0;
};
#endif
