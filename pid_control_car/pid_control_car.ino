#include <ArduinoJson.h>
#include <Servo.h>
#include "MotorDriver.h"
#define CUSTOM_ID "usb_rear_wheel"
#define MOTORTYPE YF_IIC_RZ  // rz7889
const int offsetm1 = 1;
const int offsetm2 = 1;
const int offsetm3 = 1;
const int offsetm4 = 1;
const int encoderPinA = 3;
volatile int encoderTicks = 0;
unsigned long lastTime = 0;
const int encoderTicksPerRevolution = 20;
MotorDriver motorDriver = MotorDriver(MOTORTYPE);
Servo servo = Servo();

void setup() {
  Serial.begin(115200);
  motorDriver.begin();
  motorDriver.motorConfig(offsetm1, offsetm2, offsetm3, offsetm4);
  motorDriver.setPWMFreq(50);
  servo.attach(10);

  pinMode(encoderPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
  lastTime = millis();
}

void encoderISR() {
  encoderTicks++;
}

void controlLeftWheels(int speed) {
  motorDriver.setSingleMotor(M3, -speed);  // rear
  motorDriver.setSingleMotor(M4, -speed);  // rear
}

void controlRightWheels(int speed) {
  motorDriver.setSingleMotor(M1, speed);  // front
  motorDriver.setSingleMotor(M2, speed);  // front
}

int wheelSpeedRatio(int speed) {
  int ratio = 100;
  int newSpeed = ratio * speed;
  return newSpeed;
}

void loop() {
  if (Serial.available()) {
    String jsonString = Serial.readString();
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      return;
    }


    if (doc.containsKey("command") && doc["command"] == "I") {
        Serial.println(CUSTOM_ID);
      } 

    else{
       JsonArray targetVelArray = doc["target_vel"];
    if (targetVelArray.size() == 2) {
      int rearWheelSpeed = wheelSpeedRatio(targetVelArray[0]);
      int frontWheelSpeed = wheelSpeedRatio(targetVelArray[1]);
      controlLeftWheels(rearWheelSpeed);
      controlRightWheels(frontWheelSpeed);
    } else {
      Serial.println("Invalid target_vel array size");
    }
    }
  }
}
