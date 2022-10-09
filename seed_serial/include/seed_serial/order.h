#ifndef ORDER_H
#define ORDER_H
#include <serial/serial.h>


struct axis_angle_msg
{
    float angle1;
    float angle2;
    float angle3;
    float angle4;
    float angle5;
    float angle6;
    float speed;
    float claw;
    axis_angle_msg();
    axis_angle_msg(float a1,float a2,float a3,float a4,float a5,float a6,float s, float cw);
};

class order
{
public:
    void move_order_init(axis_angle_msg msg);
    void control_order_init();
    size_t send(serial::Serial &sp);

private:
    void trans(float angle,uint8_t a[]);
    uint8_t order[48];
    
};

#endif
