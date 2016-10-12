#ifndef __ARDUCQUAD_H__
#define __ARDUCQUAD_H__


#define FRAME_TYPE 500
#define BOARD_VERSION 2 
//#define VERSION_STRING "Frame type: " #FRAME_TYPE ", board version: " #BOARD_VERSION


#define TIME_STEP  2500 //in microseconds
#define UPDATE_INTERVAL (TIME_STEP / 1000000.0f)


#if (FRAME_TYPE == 450)
  #define CONTROL_SCALE 0.5 //Joystik scale is -0.5 to 0.5 RAD yaw/pitch
  #define YAW_CONTROL_SCALE -2.0 //Joystick is 2.0 RAD/sec rotation
  #define YAW_TRIM 0
  #define XY_KP 100 //400
  #define XY_KD 8000
  #define XY_KI 30
  #define THROTTLE_SCALE 1.1 //linear
#elif (FRAME_TYPE == 500)
  #define CONTROL_SCALE 0.6 //Joystik scale is -0.4 to 0.4 RAD yaw/pitch
  #define YAW_CONTROL_SCALE -2.0 //Joystick is 2.0 RAD/sec rotation
  #define YAW_TRIM 50
  #define XY_KP 200 //400
  #define XY_KD 25000
  #define XY_KI 100
  #define THROTTLE_SCALE 0.6 //exponential
#endif

#define ALT_KP 10
#define ALT_KD 50
#define ALT_KI 20


#define YAW_KP 20
#define YAW_KD 0
#define YAW_KI 80 //200 //yaw PID works on I for smooth action 

//0.01: oscilation  tremble +-0.1 on  
//0.05: tremble +- 0.5
//0.1: tremble +-1.0
//0.2 tremble +-1.2
//1.0 tremble +-3.0, with new blades +-1.0
#define GYRO_LPF 0.1 //was 0.5

//0.01: no tremble
#define ACC_LPF 0.05   //was 0.1
#define MAG_LPF 0.00
#define ANGLE_LPF 0.004
#define BARO_LPF 0.05

int normalizePWM (int val);
void updateVoltage();
#endif
