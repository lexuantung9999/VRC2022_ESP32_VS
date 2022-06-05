#include <EEB.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
bool motor_init_stt = 0;


void DCMotor::Init(){
    pinMode(M1_IO1,OUTPUT); pinMode(M1_IO2,OUTPUT); pinMode(M2_IO1,OUTPUT); pinMode(M2_IO2,OUTPUT);
    pinMode(M3_IO1,OUTPUT); pinMode(M3_IO2,OUTPUT); pinMode(M4_IO1,OUTPUT); pinMode(M4_IO2,OUTPUT);  
    pwm.begin();
    pwm.setOscillatorFrequency(Clock_PCA9685);
    pwm.setPWMFreq(Motor_FREQ);
    pwm.setPWM(1, 0,0); pwm.setPWM(2, 0,0); pwm.setPWM(3, 0,0); pwm.setPWM(4, 0,0);
    motor_init_stt = 1;
}

void DCMotor::Run(int motor_number, int16_t pwm_input, bool dir){
    if(dir==1) {
        IN1 = 0;  IN2 = pwm_input;
        IN3 = 0;  IN4 = 0;
    }
    else{
        IN1 = 0;  IN2 = 0;
        IN3 = 0;  IN4 = pwm_input;  
    }
    pwm.setPWM(Motor_A[motor_number-1], IN1,IN2);
    pwm.setPWM(Motor_B[motor_number-1], IN3,IN4);
}

void DCMotor::Stop(int motor_number){
    IN1 = 0; IN2 = 4096; IN3 = 0; IN4 = 4096;
    pwm.setPWM(Motor_A[motor_number-1], IN1,IN2);
    pwm.setPWM(Motor_B[motor_number-1], IN3,IN4);
}
    
void DCMotor::Lift(int motor_number, int status, int16_t pwm_input){

    switch(status){
        case LIFT_UP:
            pwm.setPWM(Motor_A[motor_number-1], 0, pwm_input);
            pwm.setPWM(Motor_B[motor_number-1], 0, 0);
            lift_stt = 1;
        break;

        case LIFT_DOWN:
            pwm.setPWM(Motor_A[motor_number-1], 0, 0);
            pwm.setPWM(Motor_B[motor_number-1], 0, pwm_input);
            lift_stt = -1;
        break;

        case LIFT_STOP:
            pwm.setPWM(Motor_A[motor_number-1], 0, 4096);
            pwm.setPWM(Motor_B[motor_number-1], 0, 4096);
            lift_stt = 0;
        break;
    }
} //up, down or stop
void Servo_Motor::Init(){
    if(!motor_init_stt){
        pwm.begin();
        pwm.setOscillatorFrequency(Clock_PCA9685);
        pwm.setPWMFreq(Motor_FREQ);
    }
}

void Servo_Motor::Angle(int angle, int servo_num){
    T_on = map(angle,0,180,1,2); // T_on in ms
    pwm.writeMicroseconds(My_servo[servo_num-1], T_on*1000);
    //pwm_val = (int) (T_on/(Ts/4096));
    //pwm.setPWM(My_servo[servo_num-1],0,pwm_val);
}

void Servo_Motor::Stop(int servo_num){
    pwm.setPWM(My_servo[servo_num-1],4096,4096);
}

void scan_i2c(){
    byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
 
  delay(5000);           // wait 5 seconds for n

}