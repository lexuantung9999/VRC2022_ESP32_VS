#ifndef _EEB_h_
#define _EEB_h_

// This library define I/O of ESP32 Expansion Board (EEB), made by tunglx
// IO control dirrection of motor 
// Kinematics of different drive robot
#if ARDUINO > 22
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#define M1_IO1  15
#define M1_IO2  2
#define M2_IO1  4
#define M2_IO2  16
#define M3_IO1  17
#define M3_IO2  5
#define M4_IO1  18
#define M4_IO2  19

#define LIFT_UP     1
#define LIFT_DOWN   -1
#define LIFT_STOP   0

#define M1_A  8
#define M1_B  9
#define M2_A  10
#define M2_B  11
#define M3_A  12
#define M3_B  13
#define M4_A  14
#define M4_B  15

#define Servo1 2
#define Servo2 3
#define Servo3 4
#define Servo4 5
#define Servo5 6
#define Servo6 7

#define Motor_FREQ  50
#define Clock_PCA9685 27000000

class DCMotor
{
private:
    /* data */
    int16_t IN1,IN2,IN3,IN4; // 4 value to control dc motor using setPWM function of PCA9685 library
    int Motor_A[4] = {M1_A,M2_A,M3_A,M4_A}, Motor_B[4]={M1_B,M2_B,M3_B,M4_B};

public:
    void Init();
    /*!
    *  @brief  Note: motor 4 using lift mechanism and motor 3 using rotate mechanism
    *  @param  motor_number number of motor
    *  @param  pwm_input pwm to control motor, 0-4096
    *  @param  dir direction of motor, 0: clockwise 1:counterclockwise
    */
    void Run(int motor_number, int16_t pwm_input, bool dir); 
    
    /*!
    *  @brief  Stop lift
    */
    void Stop(int motor_number);

      /*!
    *  @brief  Note: motor 4 using lift mechanism and motor 3 using rotate mechanism
    *  @param  motor_number number of motor
    *  @param  status: LIFT_UP, LIFT_DOWN or LIFT_STOP
    *  @param  pwm_input pwm to control motor, 0-4096
    */
    void Lift(int motor_number, int status, int16_t pwm_input); //up, down or stop
};


class Servo_Motor
{
private:
   int My_servo[6]={Servo1,Servo2,Servo3,Servo4,Servo5,Servo6};
   int pwm_val, T_on;
   float T_ON_90 = 1.5, T_ON_0 = 1.0, T_ON_180 =2.0, Ts=20; // T_ON_90 is time in ms to control servo in 90 degree.Ts is cycle of PWM control servo: 20ms ~ freq: 50Hz

public:
   void Init();

    /*!
    *  @brief  Control angle of servo function
    *  @param  angle angle of motor, 0-180. If servo 360, 0 is CW, 180 is CCW, 90 is stop
    *  @param  servo_num  number of servo  
    */
   void Angle(int angle, int servo_num);  
   void Stop(int servo_num);

};


#endif