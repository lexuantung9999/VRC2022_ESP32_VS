#include <Arduino.h>
#include <EEB.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

DCMotor     VRC_Motor;
Servo_Motor VRC_Servo;

void setup() {
  // put your setup code here, to run once:
  VRC_Motor.Init();
  VRC_Servo.Init();
} 

void loop() {
  // put your main code here, to run repeatedly:

  //example control motor:
  VRC_Motor.Run(1,2500,1);
  VRC_Servo.Angle(0,1);

}

