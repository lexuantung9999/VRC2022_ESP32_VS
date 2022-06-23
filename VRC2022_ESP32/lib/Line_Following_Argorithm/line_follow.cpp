#include "line_follow.h"

// *************************************Line Following ******************************************************* //
//************ Line*****    1       2       3       4        5          Error
//************ Value****    0       0       0       0       1           -4
//                          0       0       0       1       1           -3                          
//                          0       0       0       1       0           -2
//                          0       0       1       1       0           -1       
//                          0       0       1       0       0           0
//                          0       1       1       0       0           1
//                          0       1       0       0       0           2
//                          1       1       0       0       0           3
//                          1       0       0       0       0           4
// 
// 0 - 255: 0-5V -> 5V~255 0V~0 2.5V~128  
// pwm cố định = 100;
// pwm + k*Err; 
// x = k*Err
// float k = 10.5;
// Err = 4 -> x=200  -> 300 -100
// analogWrite(10, 100);

// calculate_output_control(100, 6.0, digitalRead(1), digitalRead(2), ... );
//  analogWrite(10, pwm_left);
// analogwrite(9,pwm_right);


void line_follow::calculate_output_control(int16_t base_speed, float Kp, float Kd, bool input1, bool input2, bool input3, bool input4, bool input5){

    if(input5==1 && input4==0)                           Err =-4;
    else if(input4==1 && input5==1 && input3==0)         Err =-3;
    else if(input4==1 && input5==0 && input3==0)         Err =-2; 
    else if(input3==1 && input4==1 && input5==0)         Err =-1;
    
    else if(input3==1 && input4==0 && input2==0)         Err =0;

    // else if(input3==1 && input4==1 && input5==1){
    //     Err =0; 
    //     cross = 1;
    // }
    //     // cros, skip cross line
    // else if(input3==1 && input4==1 && input2==1){
    //     Err =0;
    //     cross = -1;
    // }

    else if(input1==1 && input2==0)                      Err =4;
    else if(input1==1 && input2==1 && input3==0)         Err =3;
    else if(input2==1 && input3==0 && input1==0)         Err =2; 
    else if(input2==1 && input3==1 && input3==1)         Err =1;

    else if(input1==0 && input2==0 && input3==0 && input4==0 && input5==0){
        if(preErr>=0) Err=5;
        else if(preErr<0) Err = -5;
    }

    P = Kp*Err;
    D = Kd*(Err-preErr)*1000;
    output = P+D;
    left_pwm = base_speed - output;
    right_pwm = base_speed + output;
    preErr = Err;
}


