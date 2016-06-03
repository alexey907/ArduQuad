#include "ArduQuad.h"
#include "PID.h"
#include "MPU6050Lite.h"
#include "AFBMP180.h"
#include "DuePWM.h"
#include "SignalMe.h"
#include "avg.h"
#include "LPF.h"
#include "Logger.h"
#include "Battery.h"
#include "BaroAltHold.h"


#include <Servo.h>


Battery batt;

Servo escFL;
Servo escFR;
Servo escRR;
Servo escRL;

#define CHECK_RX 1

PID x_PID(XY_KP, XY_KD, XY_KI);
PID y_PID(XY_KP, XY_KD, XY_KI);
PID yaw_PID(YAW_KP, YAW_KD, YAW_KI);


MPU6050Lite mpu;
AFBMP180 baro;
BaroAltHold baroAltHold(&baro, &mpu);

CREATE_PWM(chX, 28)
CREATE_PWM(chY, 27)
CREATE_PWM(chTrottle, 30)
CREATE_PWM(chYaw, 32)
CREATE_PWM(chAltHold, 34)
CREATE_PWM(chAltSet, 34)


void doVibroTest();

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


  l.println("Initializing ESC");
#ifdef F450
  escFR.attach(12);
  escFL.attach(13);
  escRR.attach(10);
  escRL.attach(11);
#endif
#ifdef S500
  escFR.attach(10);
  escFL.attach(13);
  escRR.attach(11);
  escRL.attach(12);
#endif

  escFR.writeMicroseconds(1000);
  escFL.writeMicroseconds(1000);
  escRR.writeMicroseconds(1000);
  escRL.writeMicroseconds(1000);
  l.println("ESC ready");

  Wire.begin();
  Wire.setClock(400000);
  
  Wire1.begin();
  Wire1.setClock(400000);
  

  l.print("Battery detected: LiPo"); l.print(batt.getCells()); l.print("S, Voltage="); l.print((float)batt.getMillivolts() / 1000.0) + l.println("V");

  l.println("Reseting MPU");

  mpu.enable(&Wire);

  //l.print("MPU Detected: "); l.println(mpu.getDeviceID());
  buzzer.play(50, 100, 2);

  l.println("Calibrating IMU");
  while (!mpu.calibrate()) {
    updateVoltage();
    greenLED.cycle(50, 50);
  }

  l.println("Calibrating Barometer");
  baro.begin(&Wire1);
  while (!baro.calibrate()) {
    updateVoltage();
    greenLED.cycle(100, 100);
  }

  buzzer.play(50, 100, 3);


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

  if (chYaw.getValue() < CH_MIN) {
    l.print("Will do a vibro test. Hold quad tight!");
    buzzer.play(500, 1000, 1);
    buzzer.play(400, 1000, 1);
    buzzer.play(300, 1000, 1);
    buzzer.play(200, 1000, 1);
    buzzer.play(100, 1000, 1);
    doVibroTest();
  }
  buzzer.play(100, 200, 4);

  lastLoop = micros();

}


int loopCount = 0;
boolean altMode = false;
void loop() {
  loopCount++;

  baro.updateReading();
  mpu.updateIMUData();
  long loopTime = micros() - lastLoop;

  while ((micros() - lastLoop) <  TIME_STEP) {
    mpu.updateIMUData();
  }


  lastLoop = micros();

  greenLED.set(chTrottle.isReady() ? HIGH : LOW);


  updateVoltage();



  float targetX =  (float)(chX.getValue() - 1500) * CONTROL_SCALE / 500.0 ;
  float targetY =  (float)(chY.getValue() - 1500) * CONTROL_SCALE / 500.0;
  float targetYaw =  (float)(chYaw.getValue() - 1500 + YAW_TRIM) * YAW_CONTROL_SCALE / 500.0;


  FVector3D* angles = mpu.getAngles();
  FVector3D* rotation = mpu.getGyro();


  
  float outputX = x_PID.compute(angles->y, -targetX);
  float outputY = y_PID.compute(angles->x, -targetY);
  float outputYaw = yaw_PID.compute(-rotation->z, -targetYaw);




  int motorPower = chTrottle.getValue();

  int altHold = baroAltHold.update(motorPower);

  if (baroAltHold.isOn()) {
    buzzer.cycle(10, 100);
  }
  
  int motorFR = normalizePWM(motorPower + outputX + outputY - outputYaw + altHold);
  int motorFL = normalizePWM(motorPower + outputX - outputY + outputYaw + altHold);
  int motorRR = normalizePWM(motorPower - outputX + outputY + outputYaw + altHold);
  int motorRL = normalizePWM(motorPower - outputX - outputY - outputYaw + altHold);

  if (motorPower < 1060) {
    motorFR = motorFL = motorRR = motorRL = 1000;
  }

  if ((loopCount % 10) == 0)
  {
    l.print(loopTime);; l.print(",\t");

    l.print(baro.getAltitude()); l.print("(");
    l.print(altHold); l.print("),\t");
    
    angles->print(&l); l.print(",\t");
    rotation->print(&l); l.print(",\t");



    l.print(outputX); l.print(",\t");
    l.print(outputY); l.print(",\t");
    l.print(outputYaw); l.print(",\t");
    l.print("\t");

    l.print(motorFL); l.print(",\t");
    l.print(motorFR); l.print(",\t");
    l.print(motorRL); l.print(",\t");
    l.print(motorRR);
    l.println("");
  }

  escFR.writeMicroseconds(motorFR);
  escFL.writeMicroseconds(motorFL);
  escRR.writeMicroseconds(motorRR);
  escRL.writeMicroseconds(motorRL);
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
int normalizePWM (int val) {
  val = (val > 2000) ? 2000 : val;
  val = (val < 1000) ? 1000 : val;
  return val;
}

#define ROUNDS 50

void doVibroTest() {

  while (chYaw.getValue() < CH_MIN) {
    Servo* pServo = NULL;


    if ( chX.getValue() > CH_MAX && chY.getValue() > CH_MAX) {
      Serial.println("Testing FR motor");
      pServo = &escFR;
    } else if ( chX.getValue() > CH_MAX && chY.getValue() < CH_MIN) {
      Serial.println("Testing FL motor");
      pServo = &escFL;
    } else if ( chX.getValue() < CH_MIN && chY.getValue() > CH_MAX) {
      Serial.println("Testing RR motor");
      pServo = &escRR;
    } else if ( chX.getValue() < CH_MIN && chY.getValue() < CH_MIN) {
      Serial.println("Testing RL motor");
      pServo = &escRL;
    } else {
      delay(100);
      continue;
    }

    for (int throttle = 1100; throttle < 2000; throttle += 50) {
      pServo->writeMicroseconds(throttle);
      float dispersionGyro = 0;
      float dispersionAccel = 0;
      FVector3D oldGyro;
      FVector3D oldAccel;
      for (int round = 0; round  < ROUNDS; round ++) {

        for (int n = 0; n < 20; n++) {
          mpu.updateIMUData();
        }
        FVector3D* accel = mpu.getAccel();
        FVector3D* gyro = mpu.getGyro();

        oldGyro.sub(*gyro);
        dispersionGyro += oldGyro.mag();
        oldGyro.set(*gyro);

        oldAccel.sub(*accel);
        dispersionAccel += oldAccel.mag();
        oldAccel.set(*accel);
      }
      if (chYaw.getValue() > CH_MIN) {
        break;
      }
      Serial.print("RPM =" ); Serial.print(throttle), Serial.print(", ");
      Serial.print("Gyro =" ); Serial.print(dispersionGyro  / ROUNDS); Serial.print(", ");
      Serial.print("Accel =" ); Serial.print(dispersionAccel / ROUNDS); Serial.println();
    }
    pServo->writeMicroseconds(1000);
  }
  escFR.writeMicroseconds(1000);
  escFL.writeMicroseconds(1000);
  escRR.writeMicroseconds(1000);
  escRL.writeMicroseconds(1000);
}

