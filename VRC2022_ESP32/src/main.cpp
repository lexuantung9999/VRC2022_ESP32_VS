#include <Arduino.h>
#include <EEB.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <main.h>
#include <PS2X_lib.h>
#include <malloc.h>
#include <math.h>


DCMotor     VRC_Motor;
Servo_Motor VRC_Servo;
PS2X        VRC_PS2;
char PS2_text[100];
int16_t pwm_left, pwm_right;
bool dir_left, dir_right;

/*!
  *  @brief  Config IO pin, endstop pin, another pin, ...
*/
void GPIO_config(){
  pinMode(MAX_END_STOP, INPUT_PULLUP); pinMode(MID_END_STOP, INPUT_PULLUP); pinMode(MID_END_STOP, INPUT_PULLUP);
  pinMode(ANOTHER1, OUTPUT); pinMode(ANOTHER2, OUTPUT); pinMode(ANOTHER3, OUTPUT); 
}

TimerHandle_t xTimers[1]; // using 1 timer

void vTimerCallback(TimerHandle_t xTimer){
    configASSERT(xTimer);
    int ulCount = (uint32_t) pvTimerGetTimerID(xTimer);
    if(ulCount==0){
       VRC_PS2.read_gamepad(0, 0);
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  VRC_Motor.Init();
  VRC_Servo.Init();
  GPIO_config();

  // Timer config
  xTimers[ 0 ] = xTimerCreate("Timer PS2",pdMS_TO_TICKS(100),pdTRUE,( void * ) 0,vTimerCallback);
  xTimerStart(xTimers[0],0);

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

void VRC_Control(){
  
  // if(VRC_PS2.Button(PSB_L2)){
    
    // val_RY = map(val_RY,0,255,4096,-4096);
    // val_LX = map(val_LX,0,255,4096,-4096);

    // pwm_left = val_RY - val_LX;
    // pwm_right = val_RY + val_LX;

    // if(abs(pwm_left)<=MIN_PWM) pwm_left = 0;
    // if(abs(pwm_right)<=MIN_PWM) pwm_right = 0;
    // if(pwm_right>MAX_PWM ) pwm_right = MAX_PWM ;
    // if(pwm_right<-MAX_PWM ) pwm_right = -MAX_PWM ;
    // if(pwm_left>MAX_PWM ) pwm_left = MAX_PWM ;
    // if(pwm_left<-MAX_PWM ) pwm_left = -MAX_PWM ;

    // if(pwm_left >=0) dir_left =0;
    // else {
    //   dir_left =1; pwm_left = -pwm_left;
    // }
    // if(pwm_right >=0) dir_right =0;
    // else {
    //   dir_right =1; pwm_right = -pwm_right;
    // }
    //sprintf(PS2_text,"pwm_left: %d, dir_left: %d  pwm_right: %d, dir_right: %d",pwm_left,dir_left,pwm_right,dir_right);
    sprintf(PS2_text,"RX: %d, LX: %d  RY: %d, LY: %d",VRC_PS2.Analog(PSS_RX),VRC_PS2.Analog(PSS_LX),VRC_PS2.Analog(PSS_RY),VRC_PS2.Analog(PSS_LY));
    //sprintf(PS2_text,"tri: %d, cros: %d, sqr: %d, circ: %d", VRC_PS2.Button(PSB_TRIANGLE),VRC_PS2.Button(PSB_CROSS),VRC_PS2.Button(PSB_SQUARE),VRC_PS2.Button(PSB_CIRCLE));
    Serial.println(PS2_text);
  //     if(VRC_PS2.Analog(PSS_RY)<100){
  //         VRC_Motor.Run(LEFT_MOTOR,2500,0);
  //         VRC_Motor.Run(RIGHT_MOTOR,2500,0);
  //     }

  //     if(VRC_PS2.Analog(PSS_RY)>135){
  //         VRC_Motor.Run(LEFT_MOTOR,2500,1);
  //         VRC_Motor.Run(RIGHT_MOTOR,2500,1);
  //     }
  //  }

  // else if (VRC_PS2.ButtonReleased(PSB_L2)){
  //   pwm_left =0; pwm_right =0; dir_left=0; dir_right=0;
  //   VRC_Motor.Run(LEFT_MOTOR,pwm_left,dir_left);
  //   VRC_Motor.Run(RIGHT_MOTOR,pwm_right,dir_right);
  // }

  // VRC_Motor.Run(LEFT_MOTOR,pwm_left,dir_left);
  // VRC_Motor.Run(RIGHT_MOTOR,pwm_right,dir_right);
  // if(VRC_PS2.Button(PSB_R1)){
  //     if(VRC_PS2.Analog(PSS_LY)>=NOISE_J_DOWN){
  //       VRC_Servo.Angle(90,PICK_UP_SERVO1);
  //       VRC_Servo.Angle(180,PICK_UP_SERVO2);
  //       Serial.println("Servo 1 2 run forward");
  //     }
  //     else if(VRC_PS2.Analog(PSS_LY)<NOISE_J_DOWN){
  //       VRC_Servo.Angle(90,PICK_UP_SERVO1);
  //       VRC_Servo.Angle(180,PICK_UP_SERVO2);
  //       Serial.println("Servo 1 2 run backward");
  //     }
  //     else {
  //       VRC_Servo.Stop(PICK_UP_SERVO1); 
  //       VRC_Servo.Stop(PICK_UP_SERVO2); 
  //       Serial.println("Servo stop");
  //     }
  // }

  // if(VRC_PS2.Button(PSB_PAD_UP)){
  //   VRC_Motor.Lift(LIFT_MOTOR,LIFT_UP,4096);
  //   Serial.println("Lift up");
  // }

  // if(VRC_PS2.Button(PSB_PAD_DOWN)){
  //   VRC_Motor.Lift(LIFT_MOTOR,LIFT_DOWN,4096);
  //   Serial.println("Lift down");
  // }

  // if(VRC_PS2.ButtonReleased(PSB_PAD_UP)||VRC_PS2.ButtonReleased(PSB_PAD_DOWN)){
  //   VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
  //   Serial.println("Lift stop");
  // }

}
void loop() {
  // put your main code here, to run repeatedly:

  //example control motor:
  /*
  VRC_Motor.Run(1,2500,1);
  VRC_Servo.Angle(1,0);
  */
  //example reading ps2:
  
  // VRC_Motor.Run(LEFT_MOTOR,2500,1);
  // delay(2000);
  // VRC_Motor.Run(LEFT_MOTOR,2500,0);
  // delay(2000);
  
  VRC_Control();
}
