#ifndef _LINE_FOLLOW_H_
#define _LINE_FOLLOW_H_

#include "stdint.h"

#define BASE_PWM_LINE   500
class line_follow
{
private:
    /* data */
public:
    float output;
    float Err=0, preErr=0;
    int16_t left_pwm, right_pwm;
    void calculate_output_control(float Kp, bool input1, bool input2, bool input3, bool input4, bool input5);
};


#endif

