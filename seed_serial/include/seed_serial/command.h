#pragma once
#include <ros/ros.h>

union numByte
{
    float num;
    uint8_t byte[4];
    numByte();
    numByte(float data)
    {
        num = data;
    }
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

    joint_cmd(float a1,float a2,float a3,float a4,float a5,float a6,float sp, float cw)
    {
        angle1.num = a1;
        angle2.num = a2;
        angle3.num = a3;
        angle4.num = a4;
        angle5.num = a5;
        angle6.num = a6;
        speed.num = sp;
        claw.num = cw;
    };

};




