#ifndef __UBLOX7NGEONAV_H__
#define __UBLOX7NGEONAV_H__

#include "GeoNav.h"
#include <arduino.h>

//#define DEBUG

#define MAX_PACKET 1024

#define MSG_POSLLH  0x0201
#define MSG_ACK     0x0105
#define MSG_SETMSG 0x0106
#define MSG_SET_DATARATE 0x0806
#define MSG_SET_PORTRATE 0x0006
#define MSG_SET_NAV 0x2406
#define MSG_SBAS 0x3201

class UBlox7nGeoNav: public IGeoNav {
  public:
    int32_t getLon();
    int32_t getLat();
    long getSignal();
    bool init();
    void updateData();
  private:
    byte m_packet[MAX_PACKET];
    int32_t m_lon = 0, m_lat = 0, m_alt = 0;
    uint32_t m_acc = 0;
    
    uint16_t m_messageIDFilter = 0;
    int  m_pos = 0;
    int  m_payloadSize = 0;
    bool recvPacket(long timeoutMicros);
    bool sendPacket(uint16_t messageID, byte* payload, uint16_t payloadSize);
    void printHex(uint8_t data);
    bool waitAck(uint16_t messageID);

};
#endif
