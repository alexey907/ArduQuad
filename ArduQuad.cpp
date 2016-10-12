#include "ArduQuad.h"
#include "PID.h"


#include "MS5611Baro.h"
#include "DuePWM.h"
#include "SignalMe.h"
#include "Logger.h"
#include "Battery.h"
#include "BaroAltHold.h"
#include "HMC5883Compass.h"
#include "UBlox7nGeoNav.h"
#include "GPSPosHold.h"

#include "Motor.h"

Battery batt;

//#define CHECK_RX 1

PID x_PID(XY_KP, XY_KD, XY_KI);
PID y_PID(XY_KP, XY_KD, XY_KI);
PID yaw_PID(YAW_KP, YAW_KD, YAW_KI);



ICompass* pCompass = NULL;//new HMC5883Compass();
IGeoNav* pGeoNav = NULL;//new UBlox7nGeoNav();


GPSPosHold* pGPSPosHold = NULL;//new GPSPosHold(pGeoNav, pCompass, pMPU);

#if (BOARD_VERSION == 1)
  #include "MPU9250Lite.h"
  #include "AFBMP180.h"
 // MPUBase* pMPU = new MPU9250Lite();
  IBaro* pBaro = new AFBMP180();
  
#elif (BOARD_VERSION == 2)
  #include "MPU6050Lite.h"
  MPUBase* pMPU = new MPU6050Lite();
  IBaro* pBaro = new MS5611Baro();
#endif
BaroAltHold* pBaroAltHold = new BaroAltHold(pBaro, pMPU);

Motor motorFR;
Motor motorFL;
Motor motorRR;
Motor motorRL;

CREATE_PWM(chX, 28)
CREATE_PWM(chY, 27)
CREATE_PWM(chTrottle, 30)
CREATE_PWM(chYaw, 32)
CREATE_PWM(chAltHold, 34)
CREATE_PWM(chAltSet, 34)



Logger l(&Serial, &Serial3);

SignalMe buzzer(46);
SignalMe greenLED(40);
SignalMe blueLED(38);




long lastLoop = 0;

#define CH_MIN 1100
#define CH_MAX 1900


void setup() {
  Serial.begin(250000);
  Serial3.begin(230400);

  l.print("Board version: "); l.println(BOARD_VERSION);
  l.print("Frame type: "); l.println(FRAME_TYPE);
  

  l.println("Initializing ESC");

  Motor::init();
  motorFR.begin(10);
  motorFL.begin(13);
  motorRR.begin(11);
  motorRL.begin(12);
  l.println("ESC ready");

  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin();
  Wire1.setClock(400000);


  l.print("Battery detected: LiPo"); l.print(batt.getCells()); l.print("S, Voltage="); l.print((float)batt.getMillivolts() / 1000.0) + l.println("V");


  l.println("Reseting MPU");

  pMPU->enable(&Wire);

  //l.print("MPU Detected: "); l.println(mpu.getDeviceID());
  buzzer.play(50, 100, 1);

  l.println("Calibrating IMU");
  while (!pMPU->calibrate()) {
    updateVoltage();
    greenLED.cycle(50, 50);
  }

  delay(500);
  
  if (pBaro) {
    buzzer.play(50, 100, 2);
    l.println("Calibrating Barometer");
    pBaro->begin(&Wire);
    while (!pBaro->calibrate()) {
      updateVoltage();
      greenLED.cycle(100, 100);
    }
    delay(1000);
  }

  if (pCompass) {
    buzzer.play(50, 100, 3);
    l.println("Calibrating compass");
    pCompass->begin(&Wire1);
    while (!pCompass->calibrate()) {
      updateVoltage();
      greenLED.cycle(100, 100);
    }
    delay(1000);
  }
  if (pGeoNav) {
    buzzer.play(50, 100, 4);
    l.println("Waiting for GPS init");
    if (!pGeoNav->init()) {
      Serial.println("Failed to start GPS");
      while (true);
    }
    while (pGeoNav->getSignal() > 1000) {
      updateVoltage();
      greenLED.cycle(100, 100);
    }
  }
  buzzer.play(50, 100, 5);

  x_PID.setLimits(-400, 400);
  y_PID.setLimits(-400, 400);
  yaw_PID.setLimits(-300, 300);

  chTrottle.setZero(1000);
#ifdef CHECK_RX
  l.println("Testing receiver channels");

  while (1) {
    updateVoltage();
    if (chTrottle.isReady()) {
      break;
    }
    greenLED.cycle(500, 500);

  }

  l.print("Receiver ready");
#endif

  buzzer.play(100, 200, 4);

  lastLoop = micros();

}


int loopCount = 0;
boolean altMode = false;
void loop() {
  loopCount++;

  pMPU->updateIMUData();


  if (pBaro) pBaro->updateReading();
  if (pCompass) pCompass->update();
  if (pGeoNav) pGeoNav->updateData();

  
  long loopTime = micros() - lastLoop;

  while ((micros() - lastLoop) <  TIME_STEP) {
    pMPU->updateIMUData();
  }


  lastLoop = micros();

  greenLED.set(chTrottle.isReady() ? HIGH : LOW);


  updateVoltage();



  float targetX =  (float)(chX.getValue() - 1500) * CONTROL_SCALE / 500.0 ;
  float targetY =  (float)(chY.getValue() - 1500) * CONTROL_SCALE / 500.0;
  float targetYaw =  (float)(chYaw.getValue() - 1500 + YAW_TRIM) * YAW_CONTROL_SCALE / 500.0;


  FVector3D* angles = pMPU->getAngles();
  FVector3D* rotation = pMPU->getGyro();

 if (pGPSPosHold && pGPSPosHold->update(targetX, targetY)){
    targetX += pGPSPosHold->getXControl();
    targetY += pGPSPosHold->getYControl();
  }

  float outputX = x_PID.compute(angles->y, -targetX);
  float outputY = y_PID.compute(angles->x, -targetY);

  float outputYaw = yaw_PID.compute(-rotation->z, -targetYaw);


  int motorPower = chTrottle.getValue() - 1000;

  int altHold = (pBaroAltHold != NULL) ? pBaroAltHold->update(motorPower) : 0;

  Motor::setArmed(motorPower > 20);

  motorFR.setThrottle(motorPower + outputX + outputY - outputYaw + altHold);
  motorFL.setThrottle(motorPower + outputX - outputY + outputYaw + altHold);
  motorRR.setThrottle(motorPower - outputX + outputY + outputYaw + altHold);
  motorRL.setThrottle(motorPower - outputX - outputY - outputYaw + altHold);



  if ((loopCount % 10) == 0)
  {
    l.print(loopTime);; l.print(",\t");

    if (pBaro) l.print(pBaro->getAltitude()); l.print("(");

    l.print(altHold); l.print(",\t");

    if (pCompass) {l.print(pCompass->getHeading()); l.print(", \t");}

    angles->print(&l); l.print(",\t");
    pMPU->getAccel()->print(&l); l.print(",\t");
    pMPU->getWorldAccel()->print(&l); l.print(",\t");



    l.print(outputX); l.print(",\t");
    l.print(outputY); l.print(",\t");
    l.print(outputYaw); l.print(",\t");
    l.print("\t");

    l.print(motorPower); l.print(",\t");
    l.print(motorFL.getThrottle()); l.print(",\t");
    l.print(motorFR.getThrottle()); l.print(",\t");
    l.print(motorRL.getThrottle()); l.print(",\t");
    l.print(motorRR.getThrottle());
    l.println("");
  }


}

void updateVoltage() {
  int used = batt.getUsed();
  int remaining = batt.getRemaining();
  if (used <= 0) {
    blueLED.set(HIGH);
    buzzer.set(LOW);
  } else if (remaining <= 0) {
    blueLED.cycle(50, 50);
    buzzer.cycle(50, 50);
  } else {
    blueLED.cycle(remaining, used);
    buzzer.set(LOW);
  }

}



