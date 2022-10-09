#pragma once
#include <ros/ros.h>

union numByte
{
    float num;
    uint8_t byte[4];
    numByte(float data)
    :num(data)
    { }
};

struct joint_cmd
{
    numByte angle1;
    numByte angle2;
    numByte angle3;
    numByte angle4;
    numByte angle5;
    numByte angle6;
    numByte speed;
    numByte claw;

    joint_cmd(float a1, float a2, float a3, float a4, float a5, float a6, float sp, float cw)
     : angle1(a1), angle2(a2), angle3(a3), angle4(a4), angle5(a5), angle6(a6), speed(sp), claw(cw)
    {
    }
};
