#ifndef _MPU6050Lite_H_
#define _MPU6050Lite_H_
#include <Arduino.h>
#include <Wire.h>

#include "ArduQuad.h"
#include "Vector.h"
#include "I2CDevice.h"
#include "MPUBase.h"


#define LPF_FACTOR_ACC 0.02
#define LPF_FACTOR_GYRO 0.1
#define LPF_FACTOR_MAG 0.1
#define GYRO_TO_RADPS 0.000532632218 // (1000 / 32768) * (pi/180)

#define UPDATE_INTERVAL (TIME_STEP / 1000000.0)

#define ACC_1G 8192.0

#define CAL_STEP 200
#define CAL_TRESHOLD 7


#define MPU_ADDR    0x68 // I2C address of the MPU-6050
#define AK8963_ADDR 0x0C // I2C address of magnetometer


#define REG_XA_OFFS_H        0x06 //MPU9250 0x06 //[15:0] XA_OFFS
#define REG_XA_OFFS_L        0x07 //MPU9250 0x06 //[15:0] XA_OFFS
#define REG_YA_OFFS_H        0x08 //MPU9250 0x08 //[15:0] YA_OFFS
#define REG_YA_OFFS_L        0x09 //MPU9250 0x08 //[15:0] YA_OFFS
#define REG_ZA_OFFS_H        0x0A //MPU92500x0A //[15:0] ZA_OFFS
#define REG_ZA_OFFS_L        0x0B //MPU92500x0A //[15:0] ZA_OFFS


#define REG_XG_OFFS_H        0x13
#define REG_XG_OFFS_L        0x14
#define REG_YG_OFFS_H        0x15
#define REG_YG_OFFS_L        0x16
#define REG_ZG_OFFS_H        0x17
#define REG_ZG_OFFS_L        0x18

#define REG_SMPLRT_DIV       0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#define REG_DLP_CONFIG       0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_CONFIG2    0x1D

#define REG_INT_PIN_CFG      0x37
#define REG_SENSOR_DATA      0x3B
#define REG_PWR_MGMT_1       0x6B //0 to enable dev
#define REG_PWR_MGMT_2       0x6C //0 to enable dev

#define REG_DEVICE_ID        0x75





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


#define AK8963_RA_XOUT_L   0x03  // data
#define AK8963_RA_XOUT_H  0x04
#define AK8963_RA_YOUT_L  0x05
#define AK8963_RA_YOUT_H  0x06
#define AK8963_RA_ZOUT_L  0x07
#define AK8963_RA_ZOUT_H  0x08
#define AK8963_RA_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_RA_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_RA_ASTC      0x0C  // Self test control
#define AK8963_RA_I2CDIS    0x0F  // I2C disable
#define AK8963_RA_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_RA_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_RA_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value


#define MFS_14BITS  0 // 0.6 mG per LSB
#define MFS_16BITS  1   // 0.15 mG per LSB
#define MFS_MODE_8HZ 0x02 //8Hz
#define MFS_MODE_100HZ 0x06 //100Hz continuous magnetometer data read




class MPU6050Lite: public MPUBase {
  public:
    virtual void setXAccelOffset(int16_t offset);
    virtual void setYAccelOffset(int16_t offset);
    virtual void setZAccelOffset(int16_t offset);
    virtual void setXGyroOffset(int16_t offset);
    virtual void setYGyroOffset(int16_t offset);
    virtual void setZGyroOffset(int16_t offset);
    virtual void getMotionDataRaw(FVector3D* acc, FVector3D* gyro);
    virtual void setFullScaleAccelRange(uint8_t range);
    virtual void setFullScaleGyroRange(uint8_t range);
    virtual void setPowerMode(uint8_t pwrMode);
    virtual void setSampleRate(uint8_t sampleRateDiv);
    virtual void setGyroDLPFConfig(uint8_t dlp);
    virtual void setAccelDLPFConfig(uint8_t dlp);
    virtual uint8_t getDeviceID();

};

#endif /* _MPU9250Lite_H_ */
