#include "Battery.h"

Battery::Battery() {
  m_nCells == -1;
  m_lpfVoltage.setFeedback(0.01);

}

int Battery::getUsed() {
  int cells = getCells();
  return (cells == 0) ? 0 : (cells * LIPO1S_MAX - getMillivolts());
}

int Battery::getRemaining() {
  int cells = getCells();
  return (cells == 0) ? 1000 :(getMillivolts() - cells * LIPO1S_MIN);
}


int Battery::getCells() {
  if (m_nCells == -1) {

    for (int n = 0; n < 200; n++){
      //LPF warmup
      getMillivolts();
      delay(10);
    }
    m_nCells = 0;
    int millivolts = getMillivolts();
    if(millivolts > USB_MAX){
      while ((m_nCells * LIPO1S_MAX) < millivolts) {
        m_nCells++;
  
      }
    }
  }
  return m_nCells;
}
int Battery::getMillivolts() {
  return  m_lpfVoltage.filter(1778L * (long)analogRead(0) / 100L); //((3.3 * 1.21 * 989.0) / (218.0*1024) )
}

