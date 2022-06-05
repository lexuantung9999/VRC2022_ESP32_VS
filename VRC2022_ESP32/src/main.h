#ifndef _MAIN_H_
#define _MAIN_H_


//pin can be used GPIO, define your code here
/*
#define  pin1      18
#define  pin2      19
#define  pin3      23
#define  pin4      5
#define  pin5      27
#define  pin6      26
#define  pin7      33
#define  pin8      34
#define  pin9      35

*/
//define pin to communicate with PS2 
#define PS2_DAT 12 //MISO  19
#define PS2_CMD 13 //MOSI  23
#define PS2_SEL 15 //SS     5
#define PS2_CLK 14 //SLK   18
#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false


//GPIO input output expansion
#define MAX_END_STOP 26     // using for lift endstop
#define MIN_END_STOP 27
#define ANOTHER1   5       // modify your name pin define here
#define ANOTHER2   23
#define ANOTHER3   19

//MODE
#define AUTO        1
#define MANUAL      0
//LED
#define LED_PIN          19
#define NUM_LEDS         10


//mid joystick has value: 128 -> offset it to safe in control
#define NOISE_J_UP      140
#define NOISE_J_DOWN    116

// #define MAX_PWM 800
#define MIN_PWM 50

#define LEFT_MOTOR      1
#define RIGHT_MOTOR     2
#define LIFT_MOTOR      3
#define ROTATE_MOTOR    4

#define PICK_UP_SERVO1   3
#define PICK_UP_SERVO2   4

#define SERVO1  1
#define SERVO2  2

// lift status
#define LIFT_UP     1
#define LIFT_DOWN   -1
#define LIFT_STOP   0

// pick up stt
#define PICK_UP     1
#define PICK_DOWN   -1
#define PICK_STOP   0

//ROTATE stt
#define ROTATE_WINDMILL_ON   1
#define ROTATE_WINDMILL_OFF  0

#endif
