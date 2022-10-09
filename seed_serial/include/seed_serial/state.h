#ifndef STATE_H
#define STATE_H
#include <serial/serial.h>

struct position_msg
{
    int16_t x;
    int16_t y;
    int16_t z;
    position_msg();
    position_msg(int16_t x_,int16_t y_,int16_t z_);
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
    void recieve(serial::Serial &sp,back_info_type type);
    void get_position(position_msg &msg);
    void trans(int16_t &value,uint8_t a[]);

private:
    uint8_t state[9];
};

#endif