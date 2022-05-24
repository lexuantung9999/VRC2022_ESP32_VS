#include <Arduino.h>
#include <EEB.h>
#include <Adafruit_PWMServoDriver.h>
#include <main.h>
#include <PS2X_lib.h>
#include <malloc.h>
#include <math.h>
#include <FastLED.h>
#include <MPU6050.h>
#include <I2Cdev.h>

#define GAMEPAD_LOG_INFO  0

#define LED_PIN          26
#define NUM_LEDS         10


DCMotor         VRC_Motor;
Servo_Motor     VRC_Servo;
PS2X            VRC_PS2;
CRGB            VRC_leds[NUM_LEDS];
MPU6050         VRC_MPU6050;

char PS2_text[100];
int16_t pwm_left, pwm_right;
bool dir_left, dir_right;
int stt_servo =0;
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

    //timer 0 reading gamepad
    if(ulCount==0){
       // Task 1
       VRC_PS2.read_gamepad(0, 0);
    }

    // if(ulCount==1){
    //    /*
    //    Task2
    //    */
    // }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  VRC_Motor.Init();
  VRC_Servo.Init();
  GPIO_config();
  
  // MPU config
  VRC_MPU6050.initialize();
  VRC_MPU6050.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  VRC_MPU6050.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  // led config
  FastLED.addLeds<WS2812, LED_PIN, GRB>(VRC_leds, NUM_LEDS);

  // Timer config
  xTimers[ 0 ] = xTimerCreate("Timer PS2",pdMS_TO_TICKS(100),pdTRUE,( void * ) 0,vTimerCallback);
  xTimerStart(xTimers[0],0);
  
  // xTimers[ 1 ] = xTimerCreate("Task2",pdMS_TO_TICKS(1000),pdTRUE,( void * ) 0,vTimerCallback);
  // xTimerStart(xTimers[0],0);

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

void led_random_test(void){
  for(int i=0;i<10;i++){
    VRC_leds[i] = CRGB(random(0,255), random(0,255), random(0,255));
    FastLED.show();
    delay(100);
  }
}

void VRC_Control(){
  
  if(VRC_PS2.Button(PSB_L2)){
    int16_t val_RY, val_LX;

    val_RY = VRC_PS2.Analog(PSS_RY);
    val_LX = VRC_PS2.Analog(PSS_LX);

    // loc nhieu
    if(val_RY>=NOISE_J_UP || val_RY<=NOISE_J_DOWN){
      val_RY = map(val_RY,0,255,MAX_PWM,-MAX_PWM);
    }
    else val_RY = 0;
    if(val_LX>=NOISE_J_UP || val_LX<=NOISE_J_DOWN){
      val_LX = map(val_LX,0,255,MAX_PWM,-MAX_PWM);
    }
    else val_LX=0;

    // tinh toan
    pwm_left = val_RY - val_LX;
    pwm_right = val_RY + val_LX;

    if(abs(pwm_left)<=MIN_PWM) pwm_left = 0;
    if(abs(pwm_right)<=MIN_PWM) pwm_right = 0;
    if(pwm_right>MAX_PWM ) pwm_right = MAX_PWM ;
    if(pwm_right<-MAX_PWM ) pwm_right = -MAX_PWM ;
    if(pwm_left>MAX_PWM ) pwm_left = MAX_PWM ;
    if(pwm_left<-MAX_PWM ) pwm_left = -MAX_PWM ;

    if(pwm_left >=0) dir_left =0;
    else {
      dir_left =1; pwm_left = -pwm_left;
    }
    if(pwm_right >=0) dir_right =0;
    else {
      dir_right =1; pwm_right = -pwm_right;
    }
  }

  else if (VRC_PS2.ButtonReleased(PSB_L2)){
    pwm_left =0; pwm_right =0; dir_left=0; dir_right=0;
  }
  
  #if GAMEPAD_LOG_INFO
    sprintf(PS2_text,"pwm_left: %d, dir_left: %d  pwm_right: %d, dir_right: %d \n",pwm_left,dir_left,pwm_right,dir_right);
    Serial.println(PS2_text);
    sprintf(PS2_text,"RX: %d, LX: %d  RY: %d, LY: %d \n",VRC_PS2.Analog(PSS_RX),VRC_PS2.Analog(PSS_LX),VRC_PS2.Analog(PSS_RY),VRC_PS2.Analog(PSS_LY));
    Serial.println(PS2_text);
    sprintf(PS2_text,"tri: %d, cros: %d, sqr: %d, circ: %d \n", VRC_PS2.Button(PSB_TRIANGLE),VRC_PS2.Button(PSB_CROSS),VRC_PS2.Button(PSB_SQUARE),VRC_PS2.Button(PSB_CIRCLE));
    Serial.println(PS2_text);
  #endif 

  if(VRC_PS2.Analog(PSS_LY)<=NOISE_J_DOWN){
    //Pick up box
    VRC_Servo.Angle(180,PICK_UP_SERVO1);
    VRC_Servo.Angle(0,PICK_UP_SERVO2);
    stt_servo = 1;
    //Serial.println("Pick up box");
  }
  else if(VRC_PS2.Analog(PSS_LY)>=NOISE_J_UP){
    //Remove box
    VRC_Servo.Angle(0,PICK_UP_SERVO1);
    VRC_Servo.Angle(180,PICK_UP_SERVO2);
    stt_servo = -1;
    //Serial.println("Remove box");
  }
  else {
    if(stt_servo!=0){
      VRC_Servo.Stop(PICK_UP_SERVO1); 
      VRC_Servo.Stop(PICK_UP_SERVO2); 
      //Serial.println("Servo stop");
      stt_servo = 0;
    }
    
  }


  if(VRC_PS2.Button(PSB_PAD_UP)){
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_UP,4000);
    Serial.println("Lift up");
  }

  if(VRC_PS2.Button(PSB_PAD_DOWN)){
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_DOWN,4000);
    Serial.println("Lift down");
  }

  if(VRC_PS2.ButtonReleased(PSB_PAD_UP)||VRC_PS2.ButtonReleased(PSB_PAD_DOWN)){
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
    Serial.println("Lift stop");
  }

}

int16_t ax, ay, az, gx, gy, gz;
void loop() {
  // put your main code here, to run repeatedly:
  VRC_Control();

  VRC_Motor.Run(LEFT_MOTOR,pwm_left,dir_left);
  VRC_Motor.Run(RIGHT_MOTOR,pwm_right,dir_right);

  // VRC_MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // sprintf(PS2_text,"Accel: ax: %f, ay: %f  az: %f \n",(float)ax/4096,(float)ay/4096,(float)az/4096);
  // Serial.print(PS2_text);
  // delay(500);


  // VRC_MPU6050.Read_gyro();
  // sprintf(PS2_text,"Gyro: gx: %f, gy: %f  gz: %f \n",VRC_MPU6050.GyroX,VRC_MPU6050.GyroY,VRC_MPU6050.GyroZ);
  // Serial.print(PS2_text);

  //scan_i2c();

  //led_random_test();
}




