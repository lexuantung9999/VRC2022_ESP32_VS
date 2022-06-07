#ifndef _LINE_FOLLOW_H_
#define _LINE_FOLLOW_H_

#include "stdint.h"

class line_follow
{
private:
    /* data */
public:
    int16_t output;
    float Err=0, preErr=0;
    void calculate_output_control(float Kp, bool input1, bool input2, bool input3, bool input4, bool input5);
};


#endif

