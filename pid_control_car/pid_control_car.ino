#include <Servo.h>
#include <ArduinoJson.h>
#include "MotorDriver.h"
#define MOTORTYPE YF_IIC_RZ  // rz7889
uint8_t SerialDebug = 1; // 串口打印调试 0-否 1-是

const int offsetm1 = 1;
const int offsetm2 = 1;
const int offsetm3 = 1;
const int offsetm4 = 1;

const int encoderPinA = 3; // 假设编码器连接到引脚3
volatile int encoderTicks = 0;
unsigned long lastTime = 0;
const int encoderTicksPerRevolution = 20; // 每圈的编码器脉冲数

// Initializing motors.
MotorDriver motorDriver = MotorDriver(MOTORTYPE);
Servo servo = Servo();

String jsonString = ""; // 用于存储接收到的JSON字符串
bool jsonStarted = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Motor Drive test!");
  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50); // 控制舵机时，需要设置PWM频率 ~50
  servo.attach(10);
  delay(1000);   // wait 1s
  Serial.println("Start...");

  pinMode(encoderPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
  lastTime = millis();
}

void encoderISR() {
  encoderTicks++;
}

void controlLeftWheels(int speed) {
  motorDriver.setSingleMotor(M3, -speed);  // 控制后轮
  motorDriver.setSingleMotor(M4, -speed);  // 控制后轮
}

void controlRightWheels(int speed) {
  motorDriver.setSingleMotor(M1, speed);  // 控制前轮
  motorDriver.setSingleMotor(M2, speed);  // 控制前轮
}

int wheelSpeedRatio(int speed) {
  int ratio = 100;
  int newSpeed = ratio * speed;
  return newSpeed;
}

void printRPM() {
  // 计算 RPM
  unsigned long currentTime = millis();
  unsigned long timeDifference = currentTime - lastTime;
  if (timeDifference == 0) {
    // 避免除以0的情况
    return;
  }
  float rpm = (encoderTicks / (float)encoderTicksPerRevolution) / (timeDifference / 60000.0);

  // 打印 RPM
  Serial.print("encoderTicks: ");
  Serial.println(encoderTicks);
  Serial.print("timeDifference: ");
  Serial.println(timeDifference);
  Serial.print("Current RPM: ");
  Serial.println(rpm);

  // 重置计数器
  encoderTicks = 0;
  lastTime = currentTime;
}

void loop() {
  // 检查是否有可用的串行数据
  if (Serial.available()) {
    // 读取串行数据到字符串
    String jsonString = "";
    while (Serial.available()) {
      char ch = (char)Serial.read();
      if (!isspace(ch) || ch == '\n') {
        jsonString += ch;
      }
      delay(5); // 给串行缓冲区时间来接收更多数据
    }

    // 打印接收到的JSON字符串
    // Serial.println("Received JSON: " + jsonString);

    // 解析JSON字符串
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }

    // 访问JSON对象中的target_vel数组
    JsonArray targetVelArray = doc["target_vel"];
    if (targetVelArray.size() == 2) {
      int rearWheelSpeed = wheelSpeedRatio(targetVelArray[0]);
      int frontWheelSpeed = wheelSpeedRatio(targetVelArray[1]);

      // 打印解析的值
      // Serial.print("Rear Wheel Speed: ");
      // Serial.println(rearWheelSpeed);
      // Serial.print("Front Wheel Speed: ");
      // Serial.println(frontWheelSpeed);

      // 控制电机
      controlLeftWheels(rearWheelSpeed);
      controlRightWheels(frontWheelSpeed);
    } else {
      Serial.println("Invalid target_vel array size");
    }

    // 清空jsonString，准备接收下一条数据
    jsonString = "";
  }
}
