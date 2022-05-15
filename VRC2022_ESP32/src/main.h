#ifndef _MAIN_H_
#define _MAIN_H_

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
#define MAX_END_STOP 25     // using for lift endstop
#define MID_END_STOP 32
#define MIN_END_STOP 36
#define ANOTHER1    0       // modify your name pin define here
#define ANOTHER2    2
#define ANOTHER3    39



#endif
