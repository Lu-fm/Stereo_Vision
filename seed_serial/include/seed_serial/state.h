#ifndef STATE_H
#define STATE_H
#include <serial/serial.h>

struct position_msg
{
    int16_t x;
    int16_t y;
    int16_t z;
    position_msg();
    position_msg(int16_t x_, int16_t y_,int16_t z_);
};

struct angle_msg
{
    int16_t angle1;
    int16_t angle2;
    int16_t angle3;    
    int16_t angle4;
    int16_t angle5;
    int16_t angle6;
    angle_msg();
};

class state
{
public:
    typedef enum
    {
        coordinate_value=0,
        axis_angle1_value=1,
        axis_angle2_value=2,
        ws_coordinate=3,
        state_info1=4,
        state_info2=5,
        state_info3=100,
        state_info4=102,
        state_info5=103,
    }back_info_type;
    bool recieve(serial::Serial &sp,back_info_type type);
    void get_cart_pos(position_msg &msg);
    void get_joint_pos();
    void trans(int16_t &value,uint8_t a[]);
    float joint_pos[6];
    float cart_pos[3];
    int jointFlag = 0;
    float box_pose[6]; // x, y, z, r, p, y
    ros::Publisher jointPub;
private:
    uint8_t value[9];
    int16_t tempdata;
};

#endif