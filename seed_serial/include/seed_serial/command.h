#pragma once
#include <ros/ros.h>

union bnf32
{
    float num;
    uint8_t byte[4];
    bnf32(float data)
    :num(data)
    { }
};

struct joint_cmd
{
    bnf32 angle1;
    bnf32 angle2;
    bnf32 angle3;
    bnf32 angle4;
    bnf32 angle5;
    bnf32 angle6;
    bnf32 speed;
    bnf32 claw;

    joint_cmd(float a1, float a2, float a3, float a4, float a5, float a6, float sp, float cw)
     : angle1(a1), angle2(a2), angle3(a3), angle4(a4), angle5(a5), angle6(a6), speed(sp), claw(cw)
    {
    }
};
