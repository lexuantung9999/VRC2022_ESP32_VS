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
#include <line_follow.h>

#define GAMEPAD_LOG_INFO  0
#define TEST_CASE 1 
DCMotor         VRC_Motor;
Servo_Motor     VRC_Servo;
PS2X            VRC_PS2;
CRGB            VRC_leds[NUM_LEDS];
MPU6050         VRC_MPU6050;
line_follow     VRC_line_follow;

char PS2_text[100];
int16_t pwm_left, pwm_right;
bool dir_left, dir_right;

int stt_servo = 0;
int pick_up_stt = 0, rotate_stt = 0;
bool holder_stt=0;
bool mode;

#ifndef MAX_PWM
  uint16_t MAX_PWM = 700;
  uint16_t MAX_LIFT = 1600;
#endif

/*!
  *  @brief  Config IO pin, endstop pin, another pin, ...
*/
void GPIO_config(){
  pinMode(MAX_END_STOP, INPUT_PULLUP); pinMode(MIN_END_STOP, INPUT_PULLUP); 
  pinMode(ANOTHER1, OUTPUT); pinMode(ANOTHER2, OUTPUT); pinMode(ANOTHER3, OUTPUT); 
  for(int i=0;i<5;i++){
    pinMode(line[i],INPUT);
  }
}

int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
int16_t ax, ay, az, gx, gy, gz;
float off_set_ax, off_set_ay, off_set_az;
float off_set_gx, off_set_gy, off_set_gz;
float angle_gyro_x, angle_gyro_y, angle_gyro_z;
float angle_acc_x, angle_acc_y, angle_acc_z;
float angle_x, angle_y, angle_z;

int timer_1 = 50;
void IMU_calculate_offset(void){
	int f  = 500;
	for(int i=0;i<f;i++){
      VRC_MPU6050.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

			// off_set_ax += (float) raw_ax/4096;
			// off_set_ay += (float) raw_ay/4096;
			// off_set_az += (float) raw_az/4096-1;
		
			// off_set_gx += (float) raw_gx/500;
			// off_set_gy += (float) raw_gy/500;
			off_set_gz += (float) raw_gz/500;
      
      // sprintf(PS2_text,"\n Accel: ax: %f, ay: %f  az: %f \n",off_set_ax,off_set_ay,off_set_az);
      // Serial.print(PS2_text);
	}
			// off_set_ax = (float) off_set_ax/f;
			// off_set_ay = (float) off_set_ay/f;
			// off_set_az = (float) off_set_az/f;
			// off_set_gx = (float) off_set_gx/f;
			// off_set_gy = (float) off_set_gy/f;
			off_set_gz = (float) off_set_gz/f;	

      // sprintf(PS2_text,"\n Offset Accel: ax: %f, ay: %f  az: %f \n",off_set_ax,off_set_ay,off_set_az);
      // Serial.print(PS2_text);
      // sprintf(PS2_text,"\n Offset Gyro: gx: %f, gy: %f  gz: %f \n",off_set_gx,off_set_gy,off_set_gz);
      // Serial.print(PS2_text);
}


TimerHandle_t xTimers[2]; // using 2 timer
float alpha = 8.8;
void vTimerCallback(TimerHandle_t xTimer){
    configASSERT(xTimer);
    int ulCount = (uint32_t) pvTimerGetTimerID(xTimer);

    //timer 0 reading gamepad
    if(ulCount==0){
       // Task 1
       VRC_PS2.read_gamepad(0, 0); // khong co PS2 thi ham nay khong chay thanh cong, bi treo

    }

    //Timer 1 reading angle
    // if(ulCount==1){
    //    /*
    //    Task2
    //   //  */
    //   VRC_MPU6050.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    //   gz = (float)raw_gz/500 - off_set_gz;
    //   angle_gyro_z = (float) timer_1*gz/1000;
		// 	angle_z = (angle_z+angle_gyro_z);
    // }
}

void led_random_test(void){
  for(int i=0;i<10;i++){
    VRC_leds[i] = CRGB(random(0,255), random(0,255), random(0,255));
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void led_all_color(int red, int green, int blue){
  for(int i=0;i<10;i++){
    VRC_leds[i] = CRGB(red, green, blue);
    FastLED.show();
  }
}

void led_change_mode(){
  for(int i=0;i<10;i++){
    VRC_leds[i] = CRGB(0, 0, 255);
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  for(int i=9;i>=0;i--){
    VRC_leds[i] = CRGB(0, 255, 0);
    FastLED.show();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void pick_up_box(){
  VRC_Servo.Angle(180,PICK_UP_SERVO1);
  VRC_Servo.Angle(0,PICK_UP_SERVO2);
  stt_servo = 1;
  pick_up_stt = PICK_UP;
}

void remove_box(){
  VRC_Servo.Angle(0,PICK_UP_SERVO1);
  VRC_Servo.Angle(180,PICK_UP_SERVO2);
  stt_servo = -1;
  pick_up_stt = PICK_DOWN;
}

void stop_box(){
  VRC_Servo.Stop(PICK_UP_SERVO1); 
  VRC_Servo.Stop(PICK_UP_SERVO2); 
  pick_up_stt = PICK_STOP;
}



bool input[5];
void line_following_auto(){
  // scan sensor:
  for(int i=0;i<5;i++){
    input[i]=digitalRead(line[i]);
  }
  VRC_line_follow.calculate_output_control(15,input[0],input[1],input[2],input[3],input[4]);

  pwm_left = BASE_LINE_PWM + VRC_line_follow.output;
  pwm_right = BASE_LINE_PWM - VRC_line_follow.output;

  if(pwm_left>=MAX_PWM) pwm_left = MAX_PWM;
  if(pwm_left<=MIN_PWM) pwm_left = MIN_PWM;

  if(pwm_right>=MAX_PWM) pwm_right = MAX_PWM;
  if(pwm_right<=MIN_PWM) pwm_right = MIN_PWM;
  
  VRC_Motor.Run(LEFT_MOTOR,pwm_left,0);
  VRC_Motor.Run(RIGHT_MOTOR,pwm_right,0);
  
}

void read_line(){
    bool a[5];
  for(int i=0;i<5;i++){
    a[i] = digitalRead(line[i]);
    //Serial.print(digitalRead(line[i]));
  }
  //Serial.println();
   
  VRC_line_follow.calculate_output_control(6, a[0], a[1], a[2],a[3], a[4]);
}
int dirrection = 1;

void VRC_Control(){

  if(VRC_PS2.ButtonPressed(PSB_CIRCLE)){
    while(VRC_PS2.ButtonPressed(PSB_CIRCLE));
    mode = !mode;
    Serial.println(mode);
    led_change_mode();
    if(mode == AUTO){
      led_all_color(255,150,0);
    }
  }

    //***************************** SPEED MODE ******************************//
    if(VRC_PS2.ButtonPressed(PSB_TRIANGLE)){
      while(VRC_PS2.ButtonPressed(PSB_TRIANGLE));
      MAX_PWM = MAX_PWM*2;
      //MAX_LIFT = MAX_LIFT*2;

      if(MAX_PWM>800 && MAX_PWM<3600){
        led_all_color(255,180,0); //yellow led, middle speed
      }

      if(MAX_PWM>=3600){
        MAX_PWM = 3600;
        led_all_color(221,160,221); // violet led , max speed
      }

      // if(MAX_LIFT>=3600){
      //   MAX_LIFT=3600;
      // }

      Serial.print("MAX PWM: ");
      Serial.println(MAX_PWM);
    }
    if(VRC_PS2.ButtonPressed(PSB_CROSS)){
      while(VRC_PS2.ButtonPressed(PSB_CROSS));
      //change mode to LOW speed PWM max = 800
      //Slowest
      MAX_PWM = 800 ;
      //MAX_LIFT = 1600;
      led_all_color(255,0,0); //green led, middle speed
      //VRC_Motor.Stop(LEFT_MOTOR);
      //VRC_Motor.Stop(RIGHT_MOTOR);
      Serial.print("RESET PWM: ");
      Serial.println(MAX_PWM);
      stop_box();
      VRC_Motor.Stop(LIFT_MOTOR);

    }
    //***************************** END SPEDD MODE ****************************//
  
  // ********** dirrection Mode ***********
    if(VRC_PS2.ButtonPressed(PSB_L3)){
      dirrection = -1;
    }
    if(VRC_PS2.ButtonPressed(PSB_R3)){
      dirrection = 1;
    }
  // ********** end dirrection mode ********************************//

  // ********************************** CONTROL MODE ****************************************** //  
  if(mode == MANUAL){
    // **************************** MOVING ROBOT ALGORITHM ********************// 
    int16_t val_RY, val_RX;
    if(dirrection==1){
      val_RY = VRC_PS2.Analog(PSS_RY);
      val_RX = VRC_PS2.Analog(PSS_RX);
    }
    else{
      val_RY = VRC_PS2.Analog(PSS_LY);
      val_RX = VRC_PS2.Analog(PSS_LX);
    }
    
    // loc nhieu
    if(val_RY>=NOISE_J_UP || val_RY<=NOISE_J_DOWN){
      val_RY = map(val_RY,0,255,MAX_PWM,-MAX_PWM);
    }
    else val_RY = 0;
    if(val_RX>=NOISE_J_UP || val_RX<=NOISE_J_DOWN){
      val_RX = map(val_RX,0,255,MAX_PWM,-MAX_PWM)*ROTATE_SPEED_SCALE;
    }
    else val_RX=0;

    // tinh toan

    if(val_RY>=0){
        pwm_left = val_RY - val_RX;
        pwm_right = val_RY + val_RX;
    }

    else{
       pwm_left = val_RY + val_RX;
       pwm_right = val_RY - val_RX; 
    }


    if(abs(pwm_left)<=MIN_PWM) pwm_left = 0;
    if(abs(pwm_right)<=MIN_PWM) pwm_right = 0;

    if(pwm_right>MAX_PWM ) pwm_right = MAX_PWM ;
    if(pwm_right<-MAX_PWM ) pwm_right = -MAX_PWM ;

    if(pwm_left>MAX_PWM ) pwm_left = MAX_PWM ;
    if(pwm_left<-MAX_PWM ) pwm_left = -MAX_PWM ;

    if(pwm_left >=0) dir_left =0;
    else {
      dir_left =1; 
      pwm_left = -pwm_left;
    }
    if(pwm_right >=0) dir_right =0;
    else {
      dir_right =1; 
      pwm_right = -pwm_right;
    }
  // ********************** END AGORITHM **************** //

  
  #if GAMEPAD_LOG_INFO
    sprintf(PS2_text,"pwm_left: %d, dir_left: %d  pwm_right: %d, dir_right: %d \n",pwm_left,dir_left,pwm_right,dir_right);
    Serial.println(PS2_text);
    // sprintf(PS2_text,"RX: %d, LX: %d  RY: %d, LY: %d \n",VRC_PS2.Analog(PSS_RX),VRC_PS2.Analog(PSS_LX),VRC_PS2.Analog(PSS_RY),VRC_PS2.Analog(PSS_LY));
    // Serial.println(PS2_text);
    // sprintf(PS2_text,"tri: %d, cros: %d, sqr: %d, circ: %d \n", VRC_PS2.Button(PSB_TRIANGLE),VRC_PS2.Button(PSB_CROSS),VRC_PS2.Button(PSB_SQUARE),VRC_PS2.Button(PSB_CIRCLE));
    // Serial.println(PS2_text);
  #endif 


  // *********** Control Pick box ******************************* //
  if(VRC_PS2.ButtonPressed(PSB_L2)){
    //Pick up box
    while(VRC_PS2.ButtonPressed(PSB_L2));
    if(pick_up_stt != PICK_UP){
      pick_up_box();
      Serial.println("box up");
    }
    //Serial.println("Pick up box");
    else{
      stop_box();
      Serial.println("box stop");
    }
  }

  else if(VRC_PS2.ButtonPressed(PSB_R2)){
    //Remove box
    while(VRC_PS2.ButtonPressed(PSB_R2));
    if(pick_up_stt != PICK_DOWN){
      remove_box();
      Serial.println("box remove");
    }
    //Serial.println("Remove box");
    else{
      stop_box();
      Serial.println("box stop");
    }
  }
  // ******************** End pick box*******************//

  // ******************** Control Lift ********************* //
  if(VRC_PS2.ButtonPressed(PSB_PAD_UP)){
    if(digitalRead(MAX_END_STOP) != LIFT_STOP){
      VRC_Motor.Lift(LIFT_MOTOR,LIFT_UP,MAX_LIFT);
      Serial.println("Lift up");
      //VRC_Motor.lift_stt = LIFT_UP;
    }
  }

  if(VRC_PS2.ButtonPressed(PSB_PAD_DOWN)){
    if(digitalRead(MIN_END_STOP) != LIFT_STOP){
      VRC_Motor.Lift(LIFT_MOTOR,LIFT_DOWN,300);
      Serial.println("Lift down");
      pick_up_box();
      //VRC_Motor.lift_stt = LIFT_DOWN;
    }
  }

  if(VRC_PS2.ButtonPressed(PSB_SQUARE)){
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
    Serial.println("Lift stop");
    //VRC_Motor.lift_stt = LIFT_STOP;
  }

  //******** Lift up down to pick box**********//
  if(VRC_PS2.Button(PSB_PAD_LEFT)){
    while(digitalRead(MIN_END_STOP) != LIFT_STOP){
      pick_up_box();
      VRC_Motor.Lift(LIFT_MOTOR,LIFT_DOWN,LIFT_DOWN_PWM);
      // move down
    }
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
    vTaskDelay(pdTICKS_TO_MS(500));
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_UP,MAX_LIFT);
    vTaskDelay(pdTICKS_TO_MS(1500));
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
    stop_box();
  }


  // ************************ End Control Lift *******************//

  // **************** Safe endstop lift up and down ************* //
  if(VRC_Motor.lift_stt==LIFT_UP){
    if(digitalRead(MAX_END_STOP)==0){
      VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
      Serial.println("Lift stop");
      //VRC_Motor.lift_stt = LIFT_STOP;
    }
  }

  if(VRC_Motor.lift_stt== LIFT_DOWN){
    if(digitalRead(MIN_END_STOP)==0){
      VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
      Serial.println("Lift stop");
      //VRC_Motor.lift_stt = LIFT_STOP;
    }
  }
  // ************ Safe function ***************** //


  //**************** Rotate windmill ****************** //
  if(VRC_PS2.ButtonPressed(PSB_L1)){
    while(VRC_PS2.ButtonPressed(PSB_L1));
    //Rotate windmill
    if(rotate_stt == ROTATE_WINDMILL_OFF){
      VRC_Motor.Run(ROTATE_MOTOR,4000,1);
      rotate_stt = ROTATE_WINDMILL_ON;
      Serial.println("Rotate ON");
    }

    else {
      VRC_Motor.Stop(ROTATE_MOTOR);
      rotate_stt = ROTATE_WINDMILL_OFF;
      Serial.println("Rotate OFF");
    }
  }

  //*************** End rotate winmill **************** //

  //**************** control box Holder ****************** //
  if(VRC_PS2.ButtonPressed(PSB_R1)){
    while(VRC_PS2.ButtonPressed(PSB_R1));
    //Open
    if(holder_stt == HOLD_OFF){
      VRC_Servo.Angle(180,HOLDER_SERVO);
      VRC_Servo.Angle(60,HOLDER_SERVO2);
      holder_stt=!holder_stt;
      Serial.println("Open ON");
    }
    //close
    else {
      VRC_Servo.Angle(60,HOLDER_SERVO);
      VRC_Servo.Angle(180,HOLDER_SERVO2);
      holder_stt=!holder_stt;
      Serial.println("Close OFF");
    }
  }

  //*************** End control box holder **************** //
  
  }

  else if (mode == AUTO){
    // ************************ AUTO MODE ***************************** //
    Serial.println("Auto Mode");
    // vTaskDelay(pdMS_TO_TICKS(500));
    MAX_PWM = 600;
    int i = 0;
    //increase speed
    for(i=0;i<=MAX_PWM;i+=5);
    {
      VRC_Motor.Run(LEFT_MOTOR,i,1);
      VRC_Motor.Run(RIGHT_MOTOR,i,1);
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
    //decrease speed
    for(i;i>=0;i-=5);
    {
      VRC_Motor.Run(LEFT_MOTOR,i,1);
      VRC_Motor.Run(RIGHT_MOTOR,i,1);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    VRC_Motor.Stop(LEFT_MOTOR); VRC_Motor.Stop(RIGHT_MOTOR);
    
    //pick up box
    pick_up_box();
    vTaskDelay(pdMS_TO_TICKS(3000));
    stop_box();
    vTaskDelay(pdMS_TO_TICKS(200));  

    //increase speed
    for(i=0;i<=MAX_PWM;i+=5);
    {
      VRC_Motor.Run(LEFT_MOTOR,i,0);
      VRC_Motor.Run(RIGHT_MOTOR,i,0);
      vTaskDelay(pdMS_TO_TICKS(5));
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
    //decrease speed
    for(i;i>=0;i-=5);
    {
      VRC_Motor.Run(LEFT_MOTOR,i,0);
      VRC_Motor.Run(RIGHT_MOTOR,i,0);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    VRC_Motor.Stop(LEFT_MOTOR); VRC_Motor.Stop(RIGHT_MOTOR);
    mode = MANUAL;
    led_all_color(0,255,0);

  }
}

void setup() {
  // line
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);


  // put your setup code here, to run once:
  Serial.begin(115200);
  GPIO_config();
  
  // led config
  FastLED.addLeds<WS2812, LED_PIN, GRB>(VRC_leds, NUM_LEDS);

  //config ps2:
  int err = -1;
  led_random_test();

  for(int i=0; i<10; i++){
    delay(100);
    err = VRC_PS2.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
    if(!err){
      Serial.println("Sucsessfully Connect PS2 Controller!");
      break;
    }
  }

    // MPU config
  // VRC_MPU6050.initialize();
  // IMU_calculate_offset();

    // Timer config
  xTimers[ 0 ] = xTimerCreate("Timer PS2",pdMS_TO_TICKS(100),pdTRUE,( void * ) 0,vTimerCallback);
  xTimerStart(xTimers[0],0);
  
  // xTimers[ 1 ] = xTimerCreate("Z Angle Read",pdMS_TO_TICKS(timer_1),pdTRUE,( void * ) 1,vTimerCallback);
  // xTimerStart(xTimers[1],0);
#if !TEST_CASE
  VRC_Motor.Init();
  VRC_Servo.Init();
  VRC_Servo.Angle(60,HOLDER_SERVO);
  VRC_Servo.Angle(180,HOLDER_SERVO2);

  // move lift to start postition
  while(digitalRead(MIN_END_STOP) != LIFT_STOP){
    VRC_Motor.Lift(LIFT_MOTOR,LIFT_DOWN,LIFT_DOWN_PWM);
      // move down
  }
  VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);
  vTaskDelay(pdTICKS_TO_MS(1000));
  VRC_Motor.Lift(LIFT_MOTOR,LIFT_UP,MAX_LIFT);
  vTaskDelay(pdTICKS_TO_MS(1500));
  VRC_Motor.Lift(LIFT_MOTOR,LIFT_STOP,0);

  for(int i=0;i<10;i++){
    VRC_leds[i] = CRGB(0,255,0);
    FastLED.show();
  }
  #endif 
} 



void loop() {
  // put your main code here, to run repeatedly:

// ******************* CONTROL *************** //
  // VRC_Control();
  // if(dirrection==1){
  //   VRC_Motor.Run(LEFT_MOTOR,pwm_left,dir_left);
  //   VRC_Motor.Run(RIGHT_MOTOR,pwm_right,dir_right);
  // }
  // else{
  //   VRC_Motor.Run(LEFT_MOTOR,pwm_right,!dir_right);
  //   VRC_Motor.Run(RIGHT_MOTOR,pwm_left,!dir_left);
  // }
  // ********************** END CONTROL************* //

  //Serial.println(digitalRead(MAX_END_STOP));

  // VRC_MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // sprintf(PS2_text,"Accel: ax: %f, ay: %f  az: %f \n",(float)ax/4096,(float)ay/4096,(float)az/4096);
  // //sprintf(PS2_text,"Accel: ax: %f, ay: %f  az: %f \n",angle_x,angle_y,alpha*angle_z);
  // Serial.print(PS2_text);


  // ****************** TEST LINE ********************* // 
  // bool a[5];
  // for(int i=0;i<5;i++){
  //   a[i] = digitalRead(line[i]);
  //   //Serial.print(digitalRead(line[i]));
  // }
  //Serial.println();
   
  //VRC_line_follow.calculate_output_control(6, a[0], a[1], a[2],a[3], a[4]);
  // Serial.print(VRC_line_follow.left_pwm);
  // Serial.print("  ");
  // Serial.println(VRC_line_follow.right_pwm);
  //scan_i2c();

  //led_random_test();
}




