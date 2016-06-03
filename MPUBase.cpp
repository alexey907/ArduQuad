#include "MPUBase.h"

#include "ArduQuad.h"


TrigAtan atan2_;

void MPUBase::enable(TwoWire* pWire) {
  
  m_pDevice = new I2CDevice(pWire, MPU_ADDR);
  
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  delay(200);
  digitalWrite(9, HIGH);
  delay(400);



  // reset device
  setPowerMode(PWR_MGMT_1_DEVICE_RESET);
  delay(100);
  setPowerMode(PWR_MGMT_1_CLK_ZGYRO);
  delay(100);


  setSampleRate(SMPLRT_1000HZ);

  setFullScaleAccelRange(ACCEL_FS_SEL_4G);
  setFullScaleGyroRange(BITS_GYRO_FS_1000DPS);

  setGyroDLPFConfig(G_DLPF_CFG_2100HZ_NOLPF);
  setAccelDLPFConfig(A_DLPF_CFG_NOLPF);

 // initMagnetometer();




}
/*
void MPUBase::initMagnetometer() {
  m_pDevice->writeByte(MPUBase_ADDR, REG_INT_PIN_CFG, 0x22);
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



void MPUBase::getMotionData(FVector3D* acc, FVector3D* gyro) {
  getMotionDataRaw(acc, gyro);
  acc->z += ACC_1G;
  acc->div(ACC_1G);
  gyro->mult(GYRO_TO_RADPS);

}
/*
void MPUBase::getCompasData(FVector3D* mag) {
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

void MPUBase::setGyroFilter(float gyroFilter) {
  m_gyroFilter = gyroFilter;
}
void MPUBase::setAccelFilter(float accelFilter) {
  m_accelFilter = accelFilter;
}

//function called 20-25 times each main loop to accumulate and filter readings from Acc, Gyro and Mag ADC
void MPUBase::updateIMUData() {
  FVector3D acc;
  FVector3D gyro;
  FVector3D mag;

  getMotionData(&acc, &gyro);
  //getCompasData(&mag);

  m_acc.lpf(acc, m_accelFilter);
  m_gyro.lpf(gyro, m_gyroFilter);
 // m_mag.lpf(mag, MAG_LPF);
}

//rotation speed around the axis in RAD/sec
FVector3D* MPUBase::getGyro() {
  return &m_gyro;
}

//acceleration in G
FVector3D* MPUBase::getAccel() {
  return &m_acc;
}


FVector3D* MPUBase::getAngles() {

  FVector3D gyroDelta(m_gyro);

  gyroDelta.mult(UPDATE_INTERVAL); //rotation in RADs since the last update
  m_gyroAngle.add(gyroDelta);


  float aZ2 =  m_acc.z * m_acc.z;
  FVector3D accAngle(
    atan2_.get(m_acc.y,  sqrt(aZ2 + m_acc.x * m_acc.x)),
    atan2_.get(-m_acc.x,  sqrt(aZ2 + m_acc.y * m_acc.y)),
    0);

  m_gyroAngle.lpf(accAngle, ANGLE_LPF);

  return &m_gyroAngle;
}


boolean MPUBase::calibrate () {
  FVector3D acc;
  FVector3D gyro;

  getMotionDataRaw(&acc, &gyro);
  m_calibrateAccAvg.add(acc);
  m_calibrateGyroAvg.add(gyro);


  m_calibrateCount++;
  if ((m_calibrateCount % CAL_STEP) == 0) {
    m_calibrateAccAvg.div(CAL_STEP);
    m_calibrateAccAvg.print(&Serial); 
    Serial.println();
    
    m_calibrateAccAvg.div(10);
    
    m_calibrateAccOff.sub(m_calibrateAccAvg);
    
    setXAccelOffset(m_calibrateAccOff.x);
   // Serial.print(m_calibrateAccOff.x);Serial.println();
   // Serial.print(getXAccelOffset());Serial.println();

    
    
    setYAccelOffset(m_calibrateAccOff.y);
    setZAccelOffset(m_calibrateAccOff.z);

    m_calibrateGyroAvg.div(CAL_STEP);
   // Serial.print("gyro=");m_calibrateGyroAvg.print(&Serial); Serial.println();
    
    m_calibrateGyroAvg.div(3);
    m_calibrateGyroOff.sub(m_calibrateGyroAvg);
    
    setXGyroOffset(m_calibrateGyroOff.x);
    setYGyroOffset(m_calibrateGyroOff.y);
    setZGyroOffset(m_calibrateGyroOff.z);

  
    
    if (m_calibrateAccAvg.mag() < 1.0 && m_calibrateGyroAvg.mag() < 1.0) {
      return true;
    }
    m_calibrateAccAvg.set(0, 0, 0);
    m_calibrateGyroAvg.set(0, 0, 0);

  }
  return false;

}







