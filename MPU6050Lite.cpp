#include "MPU6050Lite.h"

#include "ArduQuad.h"




void MPU6050Lite::setXAccelOffset(int16_t offset) {
  m_pDevice->writeByte(REG_XA_OFFS_H, offset >> 7);
  m_pDevice->writeByte(REG_XA_OFFS_L, (offset << 1) & 0xFF);
}

void MPU6050Lite::setYAccelOffset(int16_t offset) {
  m_pDevice->writeByte(REG_YA_OFFS_H, offset >> 7);
  m_pDevice->writeByte(REG_YA_OFFS_L, (offset << 1) & 0xFF);
}

void MPU6050Lite::setZAccelOffset(int16_t offset) {

  m_pDevice->writeByte(REG_ZA_OFFS_H, offset >> 7);
  m_pDevice->writeByte(REG_ZA_OFFS_L, (offset << 1) & 0xFF);

}

void MPU6050Lite::setXGyroOffset(int16_t offset) {
  m_pDevice->writeByte(REG_XG_OFFS_H, offset >> 8);
  m_pDevice->writeByte(REG_XG_OFFS_L, offset);
}

void MPU6050Lite::setYGyroOffset(int16_t offset) {
  m_pDevice->writeByte(REG_YG_OFFS_H, offset >> 8);
  m_pDevice->writeByte(REG_YG_OFFS_L, (offset) & 0xFF);
}

void MPU6050Lite::setZGyroOffset(int16_t offset) {

  m_pDevice->writeByte(REG_ZG_OFFS_H, offset >> 8);
  m_pDevice->writeByte(REG_ZG_OFFS_L, (offset) & 0xFF);

}
void MPU6050Lite::getMotionDataRaw(FVector3D* acc, FVector3D* gyro) {
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
void MPU6050Lite::getCompasData(FVector3D* mag) {
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



void MPU6050Lite::setPowerMode(uint8_t pwrMode) {
  m_pDevice->writeByte(REG_PWR_MGMT_1, pwrMode);
}

uint8_t MPU6050Lite::getDeviceID() {
  return m_pDevice->readByte(REG_DEVICE_ID);
}


void MPU6050Lite::setFullScaleAccelRange(uint8_t range) {
  m_pDevice->writeByte(REG_ACCEL_CONFIG, (range << 3));
}

void MPU6050Lite::setGyroDLPFConfig(uint8_t val) {
  m_pDevice->writeByte(REG_DLP_CONFIG, val);
}

void MPU6050Lite::setAccelDLPFConfig(uint8_t val) {
  m_pDevice->writeByte(REG_ACCEL_CONFIG2, val);
}

void MPU6050Lite::setSampleRate(uint8_t val) {
  m_pDevice->writeByte(REG_SMPLRT_DIV, val);
}

void MPU6050Lite::setFullScaleGyroRange(uint8_t val) {
  m_pDevice->writeByte(REG_GYRO_CONFIG, val);
}






