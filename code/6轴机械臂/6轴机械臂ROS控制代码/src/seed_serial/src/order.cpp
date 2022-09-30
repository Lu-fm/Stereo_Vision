#include <ros/ros.h>
#include <seed_serial/order.h>
/*轴角度插补指令——运动指令*/

axis_angle_msg::axis_angle_msg()
{
    angle1=-2.9;
    angle2=30.33;
    angle3=100.44;
    angle4=-0.5;
    angle5=40.94;
    angle6=-83.41;
    speed=1;
}
axis_angle_msg::axis_angle_msg(float a1,float a2,float a3,float a4,float a5,float a6,float s)
{
    angle1=a1;
    angle2=a2;
    angle3=a3;
    angle4=a4;
    angle5=a5;
    angle6=a6;
    speed=s;
}

void order::move_order_init(axis_angle_msg msg)
{
    order[0]=0xee;
    order[1]='3';
    order[2]=1;
    trans(msg.angle1,&(order[3]));
    trans(msg.angle2,&(order[7]));
    trans(msg.angle3,&(order[11]));
    trans(msg.angle4,&(order[15]));
    trans(msg.angle5,&(order[19]));
    trans(msg.angle6,&(order[23]));
    trans(1000,&(order[27]));
    trans(0,&(order[39]));
    trans(msg.speed,&(order[43]));
    order[47]=0xef;
}
/*机械臂复位控制指令——控制指令*/
void order::control_order_init()
{
    order[0]=0xfc;
    order[1]=12;
    order[2]=3;
    for(int i=3;i<=46;i++)
        order[i]=0;
    order[47]=0xfd;
}
/*float(1)->uint8_t(4)*/
void order::trans(float angle,uint8_t array[])
{
    uint8_t *p=(uint8_t*)(&angle);
    for(int i=0;i<4;i++)
    {
        array[i]=p[i];
    }
}
/*发指令*/
size_t order::send(serial::Serial &sp)
{
    sp.write(order,48);
}