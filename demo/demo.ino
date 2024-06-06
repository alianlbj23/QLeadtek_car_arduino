/***************************************************
  Motor Test - IIC Motor Drive (RZ7889 x 4)
  Servo Control

  motor driver library: https://github.com/YFROBOT-TM/Yfrobot-Motor-Driver-Library
  motor driver iic Introduction: http://www.yfrobot.com.cn/wiki/index.php?title=MotorDriver_IIC
  motor driver iic：https://item.taobao.com/item.htm?id=626324653253

  YFROBOT ZL
  08/13/2020
 ****************************************************/
#include <Servo.h>

#include "MotorDriver.h"

#define MOTORTYPE YF_IIC_RZ   // rz7889
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetm1 = 1;
const int offsetm2 = 1;
const int offsetm3 = 1;
const int offsetm4 = 1;

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);
Servo servo = Servo();

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Drive test!");
  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50); // 控制舵机时，需要设置PWM频率 ~50
  servo.attach(10);
  delay(1000);   // wait 2s
  Serial.println("Start...");
}

int angle = 0;
void loop() {
  motorDriver.servoWrite(S1, 90);
  motorDriver.servoWrite(S2, 45);
  motorDriver.servoWrite(S3, 90);
  motorDriver.servoWrite(S4, 90);
  motorDriver.servoWrite(S5, 90);
  servo.write(0);

  motorDriver.setSingleMotor(M1, 0); // 电机M1全速正转
  motorDriver.setSingleMotor(M2, 0); // 电机M2全速正转
  motorDriver.setSingleMotor(M3, -0); // 电机M3全速正转
  motorDriver.setSingleMotor(M4, -0); // 电机M4全速正转
}
