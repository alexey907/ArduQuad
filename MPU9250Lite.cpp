#include "MPU9250Lite.h"

#include "ArduQuad.h"





/*
void MPU9250Lite::initMagnetometer() {
  m_pDevice->writeByte(MPU_ADDR, REG_INT_PIN_CFG, 0x22);
  Serial.print("Magnetometer ID:");

  Serial.println(readByte(AK8963_ADDR, 0));


  m_pDevice->writeByte(AK8963_ADDR, AK8963_RA_CNTL, 0x00); // Power down magnetometer
  delay(10);
  m_pDevice->writeByte(AK8963_ADDR, AK8963_RA_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);

  float mcx =  (float)(readByte(AK8963_ADDR, AK8963_RA_ASAX) - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  float mcy =  (float)(readByte(AK8963_ADDR, AK8963_RA_ASAY) - 128) / 256. + 1.;
  float mcz =  (float)(readByte(AK8963_ADDR, AK8963_RA_ASAZ) - 128) / 256. + 1.;
  m_pDevice->writeByte(AK8963_ADDR, AK8963_RA_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  m_pDevice->writeByte(AK8963_ADDR, AK8963_RA_CNTL, MFS_16BITS << 4 | MFS_MODE_100HZ); // Set magnetometer data resolution and sample ODR

}*/
/*
int16_t MPU9250Lite::getXAccelOffset() {
  return (((m_pDevice->readByte(REG_XA_OFFS_H) << 7) & 0x7F80) |
         ((m_pDevice->readByte(REG_XA_OFFS_L) >> 1) & 0x7F));
}
*/
void MPU9250Lite::setXAccelOffset(int16_t offset) {
  m_pDevice->writeByte(REG_XA_OFFS_H, offset >> 7);
  m_pDevice->writeByte(REG_XA_OFFS_L, (offset << 1) & 0xFF);
}

void MPU9250Lite::setYAccelOffset(int16_t offset) {
  m_pDevice->writeByte(REG_YA_OFFS_H, offset >> 7);
  m_pDevice->writeByte(REG_YA_OFFS_L, (offset << 1) & 0xFF);
}

void MPU9250Lite::setZAccelOffset(int16_t offset) {

  m_pDevice->writeByte(REG_ZA_OFFS_H, offset >> 7);
  m_pDevice->writeByte(REG_ZA_OFFS_L, (offset << 1) & 0xFF);

}

void MPU9250Lite::setXGyroOffset(int16_t offset) {
  m_pDevice->writeByte(REG_XG_OFFS_H, offset >> 8);
  m_pDevice->writeByte(REG_XG_OFFS_L, offset);
}

void MPU9250Lite::setYGyroOffset(int16_t offset) {
  m_pDevice->writeByte(REG_YG_OFFS_H, offset >> 8);
  m_pDevice->writeByte(REG_YG_OFFS_L, (offset) & 0xFF);
}

void MPU9250Lite::setZGyroOffset(int16_t offset) {

  m_pDevice->writeByte(REG_ZG_OFFS_H, offset >> 8);
  m_pDevice->writeByte(REG_ZG_OFFS_L, (offset) & 0xFF);

}
void MPU9250Lite::getMotionDataRaw(FVector3D* acc, FVector3D* gyro) {
  uint8_t data[14];
  m_pDevice->readBytes(REG_SENSOR_DATA, data, 14);
  
  acc->x = (int16_t)(data[0] << 8 | data[1]);  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acc->y = (int16_t)(data[2] << 8 | data[3]);  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc->z = (int16_t)(data[4] << 8 | data[5]);  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  int tmp = data[6] << 8 | data[7];  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyro->x =  (int16_t)(data[8] << 8 | data[9]);  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro->y =  (int16_t)(data[10] << 8 | data[11]);  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro->z =  (int16_t)(data[12] << 8 | data[13]);  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

}


/*
void MPU9250Lite::getCompasData(FVector3D* mag) {
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(AK8963_RA_XOUT_L);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 7, true); // request a total of 14 registers
  mag->x =  (int16_t) (Wire.read() | Wire.read() << 8);
  mag->y =  (int16_t) (Wire.read() | Wire.read() << 8);
  mag->z =  (int16_t) (Wire.read() | Wire.read() << 8);
  uint8_t st2 = Wire.read();
  if (st2 & 0x08) {
    mag->x = mag->y = mag->z = 0;
  }
}*/



void MPU9250Lite::setPowerMode(uint8_t pwrMode) {
  m_pDevice->writeByte(REG_PWR_MGMT_1, pwrMode);
}

uint8_t MPU9250Lite::getDeviceID() {
  return m_pDevice->readByte(REG_DEVICE_ID);
}


void MPU9250Lite::setFullScaleAccelRange(uint8_t range) {
  m_pDevice->writeByte(REG_ACCEL_CONFIG, (range << 3));
}

void MPU9250Lite::setGyroDLPFConfig(uint8_t val) {
  m_pDevice->writeByte(REG_DLP_CONFIG, val);
}

void MPU9250Lite::setAccelDLPFConfig(uint8_t val) {
  m_pDevice->writeByte(REG_ACCEL_CONFIG2, val);
}

void MPU9250Lite::setSampleRate(uint8_t val) {
  m_pDevice->writeByte(REG_SMPLRT_DIV, val);
}

void MPU9250Lite::setFullScaleGyroRange(uint8_t val) {
  m_pDevice->writeByte(REG_GYRO_CONFIG, val);
}






