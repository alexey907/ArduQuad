#include "UBlox7nGeoNav.h"

int32_t UBlox7nGeoNav::getLon() {
  return m_lon;
}

int32_t UBlox7nGeoNav::getLat() {
  return m_lat;
}

long UBlox7nGeoNav::getSignal() {
  return m_acc;
}

bool UBlox7nGeoNav::init() {
  delay(2500);
  Serial.println("Configuring u-Blox GPS initial state...");
  //Generate the configuration string for Navigation Mode
  //NavMode:
  //Pedestrian Mode    = 0x03
  //Automotive Mode    = 0x04
  //Sea Mode           = 0x05
  //Airborne < 1G Mode = 0x06
  Serial.println("Setting Navigation Mode... ");
  byte navMode = 0x03;
  byte setNav[] = {0xFF, 0xFF,
                   0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                  };
  int portSpeeds[] = {115200, 4800, 9600, 19200, 38400, 57600};
  bool portSpeedOK = false;
  for (int n = 0; n < 6; n++) {
    Serial2.begin(portSpeeds[n]);
    sendPacket(MSG_SET_NAV, setNav, sizeof(setNav));
    if (waitAck(MSG_SET_NAV)) {
      portSpeedOK = true;
      Serial.print("Port speed detected:"); Serial.println(portSpeeds[n]);
      break;
    }
  }

  if (!portSpeedOK) {
    return false;;
  }

  //
  //PortRate:
  //4800   = C0 12 00
  //9600   = 80 25 00
  //19200  = 00 4B 00  **SOFTWARESERIAL LIMIT FOR ARDUINO UNO R3!**
  //38400  = 00 96 00  **SOFTWARESERIAL LIMIT FOR ARDUINO MEGA 2560!**
  //57600  = 00 E1 00
  //115200 = 00 C2 01
  //230400 = 00 84 03
  byte setPortRate[] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                        0x00, 0xC2, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
                       };
  sendPacket(MSG_SET_PORTRATE, setPortRate, sizeof(setPortRate));
  Serial2.begin(115200);

  if (!waitAck(MSG_SET_PORTRATE)) {
    return false;;
  }

  //
  //DataRate:
  //1Hz     = 0xE8 0x03
  //2Hz     = 0xF4 0x01
  //3.33Hz  = 0x2C 0x01
  //4Hz     = 0xFA 0x00
  //10Hz     = 0x64 0x00

  byte setDataRate[] = { 0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
  sendPacket(MSG_SET_DATARATE, setDataRate, sizeof(setDataRate));
  if (!waitAck(MSG_SET_DATARATE)) {
    return false;
  }

  for (int n = 0; n < 6; n++) {
    byte setNMEA[] = {0xF0, n , 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendPacket(MSG_SETMSG, setNMEA, sizeof(setNMEA));
    if (!waitAck(MSG_SETMSG)) {
      return false;
    }
  }

  byte setPOSLLH[] = {0x01, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
  sendPacket(MSG_SETMSG, setPOSLLH, sizeof(setPOSLLH));
  if (!waitAck(MSG_SETMSG)) {
    return false;
  }
  byte confSBAS[] = {0x01, 0x07, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  sendPacket(0x1606, confSBAS, sizeof(confSBAS));
  if (!waitAck(0x1606)) {
    return false;
  } 
  /*
  byte setSBAS[] = {0x01, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendPacket(MSG_SETMSG, setSBAS, sizeof(setSBAS));
  if (!waitAck(MSG_SETMSG)) {
    return false;
  }*/

  return true;
}
void UBlox7nGeoNav::updateData() {

  m_messageIDFilter = 0;

  if (!recvPacket(100)) {
    return;
  }
  uint16_t messageID = m_packet[2] | m_packet[3] << 8;
  if (messageID == MSG_POSLLH) {
    m_lon = *((int32_t*) (m_packet + 10));
    m_lat = *((int32_t*) (m_packet + 14));
    m_alt = *((int32_t*) (m_packet + 18));
    m_acc = *((uint32_t*) (m_packet + 26));
    Serial.print("POSLLH: ");Serial.print(m_lon); Serial.print(":"); Serial.print(m_lat);Serial.print(" "); Serial.print(m_alt); Serial.print(" "); Serial.println(m_acc); 
  } else if (messageID == MSG_SBAS){
    byte sbasEn = *((uint8_t*) (m_packet + 11));
    byte sbasMode = *((uint8_t*) (m_packet + 12));
    Serial.print("SBAS: "); Serial.print(sbasEn); Serial.print(", "); Serial.println(sbasMode);;
  }
  
}


bool UBlox7nGeoNav::waitAck(uint16_t messageID) {
  m_messageIDFilter = MSG_ACK;
  long t0 = millis();
  while ((millis() - t0) < 1000) {
    if (recvPacket(1000)) {
      if (((m_packet[7] << 8 | m_packet[6]) != messageID)) {
        return false;
      }
#ifdef DEBUG
      Serial.println("Got ACK!");
#endif
      return true;
    }
    delay(1);
  }

  return false;

}

bool UBlox7nGeoNav::sendPacket(uint16_t messageID, byte* payload, uint16_t payloadSize) {
  m_packet[0] = 0xB5;
  m_packet[1] = 0x62;
  m_packet[2] = messageID & 0xFF;
  m_packet[3] = messageID >> 8;
  m_packet[4] = payloadSize & 0xFF;
  m_packet[5] = payloadSize >> 8;

  for (int n = 0; n < payloadSize; n++) {
    m_packet[n + 6] = payload[n];
  }

  byte CK_A = 0, CK_B = 0;

  for (int n = 2; n < (payloadSize + 6) ; n++) {
    CK_A = CK_A + m_packet[n];
    CK_B = CK_B + CK_A;
  }

  m_packet[payloadSize + 6] = CK_A;
  m_packet[payloadSize + 7] = CK_B;


  for (int i = 0; i < (payloadSize + 8); i++) {
    Serial2.write(m_packet[i]);
  }

  Serial2.println();
  Serial2.flush();
#ifdef DEBUG
  Serial.print("Send: [");
  for (int i = 0; i < (payloadSize + 8); i++) {
    printHex(m_packet[i]);
  }
  Serial.println("], ");
#endif

}

bool UBlox7nGeoNav::recvPacket(long timeoutMicros) {

  unsigned long t0 = micros();

  static byte HDR[] = {0xB5, 0x62};
#ifdef DEBUG
  Serial.print("RCV: [");
#endif
  while (true) {

    if ((micros() - t0) > timeoutMicros) {
#ifdef DEBUG
      Serial.println("(Timeout)]");
#endif
      return false;
    }

    if (!Serial2.available()) {
#ifdef DEBUG
      Serial.println("(No Data)]");
#endif     
      return false;
    }


    byte chr = Serial2.read();

#ifdef DEBUG
    printHex(chr);
#endif

    m_packet[m_pos] = chr;
    if ((m_pos < 2) && (chr != HDR[m_pos])) { //reading packet header and message
      m_pos = 0; //reset, start from the begining
      continue;
    } else if (m_pos == 3 &&
               m_messageIDFilter != 0 &&
               m_messageIDFilter != (m_packet[2] | m_packet[3] << 8)) {
      //Serial.println("Messg id filter failed");
      m_pos = 0;
      continue;
    } else if (m_pos == 5) { //finished reading header
      m_payloadSize = m_packet[4] | m_packet[5] << 8;
    } else if (m_pos == (7 + m_payloadSize)) { //finished reading entire packet
      m_pos = 0;
      byte CK_A = 0, CK_B = 0;
      for (int n = 2; n < (m_payloadSize + 6); n++) {
        CK_A = CK_A + m_packet[n];
        CK_B = CK_B + CK_A;
      }
      if (CK_A != m_packet[6 + m_payloadSize] || CK_B != m_packet[7 + m_payloadSize]) {
#ifdef DEBUG
        Serial.println("(Invalid checksum)]");
#endif
        return false;
      };
      m_payloadSize = 0;
#ifdef DEBUG
      Serial.println("]");
#endif
      return true;
    }

    m_pos++;
  }


}



void UBlox7nGeoNav::printHex(uint8_t data) // prints 8-bit data in hex
{
  char tmp[16];
  sprintf(tmp, "%.2X ", data);
  Serial.print(tmp);
}
