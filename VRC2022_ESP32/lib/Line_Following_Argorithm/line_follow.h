#ifndef _LINE_FOLLOW_H_
#define _LINE_FOLLOW_H_

#include "stdint.h"


class line_follow
{
private:
    /* data */
public:
    float P,D;
    float output;
    float Err=0, preErr=0;
    int16_t left_pwm, right_pwm;
    int cross=0;
    void calculate_output_control(int16_t base_speed, float Kp, float Kd, bool input1, bool input2, bool input3, bool input4, bool input5);
};


#endif

