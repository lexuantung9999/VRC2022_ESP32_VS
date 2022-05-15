#include <Arduino.h>
#include <EEB.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <main.h>
#include <PS2X_lib.h>

DCMotor     VRC_Motor;
Servo_Motor VRC_Servo;
PS2X        VRC_PS2;
char PS2_text[100];

/*!
  *  @brief  Config IO pin, endstop pin, another pin, ...
*/
void GPIO_config(){
  pinMode(MAX_END_STOP, INPUT_PULLUP); pinMode(MID_END_STOP, INPUT_PULLUP); pinMode(MID_END_STOP, INPUT_PULLUP);
  pinMode(ANOTHER1, OUTPUT); pinMode(ANOTHER2, OUTPUT); pinMode(ANOTHER3, OUTPUT); 
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  VRC_Motor.Init();
  VRC_Servo.Init();
  GPIO_config();

  //config ps2:
  int err = -1;
  for(int i=0; i<10; i++){
    delay(1000);
    err = VRC_PS2.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial. print(".");
    if(!err){
      Serial.println("Sucsessfully Connect PS2 Controller!");
      break;
    }
  }
  

} 

void loop() {
  // put your main code here, to run repeatedly:

  //example control motor:
  /*
  VRC_Motor.Run(1,2500,1);
  VRC_Servo.Angle(0,1);
  */

  //example reading ps2:
  
  VRC_PS2.read_gamepad(0, 0);
  //Hold L1 button to reading analog of joystick
  if(VRC_PS2.Button(PSB_L1)){
    sprintf(PS2_text,"RY: %d RX: %d LY: %d LX: %d",VRC_PS2.Analog(PSS_RY),VRC_PS2.Analog(PSS_RX),VRC_PS2.Analog(PSS_LY),VRC_PS2.Analog(PSS_LX));
    Serial.println(PS2_text);
    delay(200);
  }
  

}
