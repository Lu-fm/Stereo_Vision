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


bool state::recieve(serial::Serial &sp,back_info_type type)
{
    
    size_t num = 9;
    // if(sp.available()<9)
    //     return false;

    size_t numRead = sp.read(value, num);
    bool doUpdate = false;

    if(value[0]==0xce && value[8]==0xcf)
    {
        // std::cout<<"number of byte: "<< num <<"\n";
        if(value[7]==state::coordinate_value)
        {   
            // get_cart_pos();
            // std::cout<<"GET POS";
            // return true;
            doUpdate = false;
        }
        else if(value[7]==state::axis_angle1_value)
        {
            jointFlag = 1;
            get_joint_pos();
            doUpdate = false;
        }

        else if(value[7]==state::axis_angle2_value && jointFlag == 1)
        {
            jointFlag = 2;
            get_joint_pos();
            doUpdate = true;
        }
    }
    return doUpdate;
    

}
/*处理coordinate_value*/
// void state::get_position(position_msg &msg)
// {
//     trans(msg.x,&(value[1]));
//     trans(msg.y,&(value[3]));
//     trans(msg.z,&(value[5]));
// }

void state::get_joint_pos()
{
    int axis = -1;
    if(jointFlag == 1)
        axis = 1;
    else if(jointFlag == 2)
        axis = 2;
    else
        return;

    for(int i = 0; i < 3; i++)
    {
        trans(tempdata, value + 2*i + 1 );
    int idx = (axis-1)*3 + i;   
        joint_pos[idx] = tempdata;
        joint_pos[idx] /= (100*180/M_PI);
        if(idx == 4)
            joint_pos[idx] -= 0;
        else if(idx == 2)
            joint_pos[idx] -= 0;
        else;
    }
}

void state::trans(int16_t &dint16,uint8_t dint8[])
{
    uint8_t *process_data=(uint8_t *)(&dint16);
    process_data[0] = dint8[1];
    process_data[1] = dint8[0];
}
