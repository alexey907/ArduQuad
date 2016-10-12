#ifndef __GPSPOSHOLD_H__
#define __GPSPOSHOLD_H__
#include "geonav.h"
#include "icompass.h"
#include "mpubase.h"


class GPSPosHold {
public:
  GPSPosHold (IGeoNav* pGeoNav, ICompass* pCompass, MPUBase* pMPU){
    m_pGeoNav = pGeoNav;
    m_pCompas = pCompass;
    m_pMPU = pMPU;
  }
  bool update(int controlX, int controlY);
  int getXControl(){
    return m_yControl;
  }
  
  int getYControl(){
    return m_yControl;
  }
  
private:
  IGeoNav* m_pGeoNav;
  ICompass* m_pCompas;
  MPUBase* m_pMPU;

  int m_nCycleCount = 0;
  int m_xControl = 0;
  int m_yControl = 0;
  
  long m_holdLon = 0;
  long m_holdLat = 0;
   
};

#endif
