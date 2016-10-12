#ifndef _MPULite_H_
#define _MPULite_H_
#include <Arduino.h>
#include <Wire.h>

#include "ArduQuad.h"
#include "Vector.h"
#include "I2CDevice.h"


#define LPF_FACTOR_ACC 0.02
#define LPF_FACTOR_GYRO 0.1
#define LPF_FACTOR_MAG 0.1
#define GYRO_TO_RADPS 0.000532632218 // (1000 / 32768) * (pi/180)



#define ACC_1G 8192.0

#define CAL_STEP 200
#define CAL_TRESHOLD 7


#define MPU_ADDR    0x68 // I2C address of the MPU-6050







#define BITS_GYRO_FS_250DPS                              0x00
#define BITS_GYRO_FS_500DPS                              0x08
#define BITS_GYRO_FS_1000DPS                             0x10
#define BITS_GYRO_FS_2000DPS                             0x18

#define DEVICE_ID_MPU9250           0x71
#define DEVICE_ID_MPU9255           0x73

#define ACCEL_FS_SEL_2G         0x00
#define ACCEL_FS_SEL_4G         0x01
#define ACCEL_FS_SEL_8G         0x02
#define ACCEL_FS_SEL_16G        0x03


#define G_DLPF_CFG_256HZ_NOLPF2                    0x00
#define G_DLPF_CFG_188HZ                             0x01
#define G_DLPF_CFG_98HZ                              0x02
#define G_DLPF_CFG_42HZ                              0x03
#define G_DLPF_CFG_20HZ                              0x04
#define G_DLPF_CFG_10HZ                              0x05
#define G_DLPF_CFG_5HZ                               0x06
#define G_DLPF_CFG_2100HZ_NOLPF                    0x07

#define A_DLPF_CFG_NOLPF                        0x00
#define A_DLPF_CFG_460HZ                             0x08
#define A_DLPF_CFG_184HZ                              0x09
#define A_DLPF_CFG_92HZ                              0x0A
#define A_DLPF_CFG_41HZ                              0x0B
#define A_DLPF_CFG_20HZ                              0x0C
#define A_DLPF_CFG_10HZ                               0x0D
#define A_DLPF_CFG_5HZ                               0x0E



#define SMPLRT_1000HZ                             0x00
#define SMPLRT_500HZ                              0x01
#define SMPLRT_250HZ                              0x03
#define SMPLRT_200HZ                              0x04
#define SMPLRT_100HZ                              0x09
#define SMPLRT_50HZ                               0x13


#define PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#define PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#define PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#define PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#define PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#define PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#define PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#define PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#define PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#define PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#define PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device




#define MFS_14BITS  0 // 0.6 mG per LSB
#define MFS_16BITS  1   // 0.15 mG per LSB
#define MFS_MODE_8HZ 0x02 //8Hz
#define MFS_MODE_100HZ 0x06 //100Hz continuous magnetometer data read


class MPUBase {
  public:

    void enable(TwoWire* pWire);
    
    //int16_t getXAccelOffset();
    
    virtual void setXAccelOffset(int16_t offset) = 0;
    virtual void setYAccelOffset(int16_t offset) = 0;
    virtual void setZAccelOffset(int16_t offset) = 0;
    virtual void setXGyroOffset(int16_t offset) = 0;
    virtual void setYGyroOffset(int16_t offset) = 0;
    virtual void setZGyroOffset(int16_t offset) = 0;
    virtual void getMotionDataRaw(FVector3D* acc, FVector3D* gyro) = 0;
    virtual void setFullScaleAccelRange(uint8_t range) = 0;
    virtual void setFullScaleGyroRange(uint8_t range) = 0;
    virtual void setPowerMode(uint8_t pwrMode) = 0;
    virtual void setSampleRate(uint8_t sampleRateDiv) = 0;
    virtual void setGyroDLPFConfig(uint8_t dlp) = 0;
    virtual void setAccelDLPFConfig(uint8_t dlp) = 0;
    virtual uint8_t getDeviceID() = 0;     
    
    void getMotionData(FVector3D* acc, FVector3D* gyro);

   // void getCompasData(FVector3D* mag);

    FVector3D* getGyro();
    FVector3D* getAccel();
    FVector3D* getAngles();
    FVector3D* getWorldAccel();
    

    void setGyroFilter(float gyroFilter);
    void setAccelFilter(float accelFilter);

    void updateIMUData();
    
    bool calibrate ();
    




  protected:
    I2CDevice* m_pDevice;
    void initMagnetometer();
   


    FVector3D m_acc;
    FVector3D m_gyro;
    FVector3D m_mag;
    FVector3D m_worldAccel;
    

    FVector3D m_gyroAngle;
    float m_gyroFilter = GYRO_LPF;
    float m_accelFilter = ACC_LPF;

    int m_calibrateCount = 0;
    FVector3D m_calibrateAccAvg;
    FVector3D m_calibrateGyroAvg;
    FVector3D m_calibrateGyroOff;
    FVector3D m_calibrateAccOff;

    void recalc();
    boolean m_needRecalc = true;;
    

};

#endif /* _MPU9250Lite_H_ */
