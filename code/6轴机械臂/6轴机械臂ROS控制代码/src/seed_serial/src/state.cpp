#include <ros/ros.h>
#include <seed_serial/state.h>
 position_msg::position_msg()
 {
    x=0;
    y=0;
    z=0;
 }
position_msg::position_msg(int16_t x_,int16_t y_,int16_t z_)
{
    x=x_;
    y=y_;
    z=z_;
}
void state::recieve(serial::Serial &sp,back_info_type type)
{
    while(true)
    {
        size_t num = sp.available();
        if(num!=0)
        {
            num = sp.read(state, num);
            if(state[0]==0xce && state[8]==0xcf && state[7]==type)
                break;
        }
    }

}
/*处理coordinate_value*/
void state::get_position(position_msg &msg)
{
    trans(msg.x,&(state[1]));
    trans(msg.y,&(state[3]));
    trans(msg.z,&(state[5]));
}
void state::trans(int16_t &value,uint8_t a[])
{
    uint8_t *p=(uint8_t *)(&value);
    p[0]=a[1];
    p[1]=a[0];
}
