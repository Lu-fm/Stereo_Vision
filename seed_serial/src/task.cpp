#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <seed_serial/seed_serial.h>
#include <std_msgs/String.h>

void wait(serial::Serial &sp, state &mystate, int pos_choose);
void get_pos(serial::Serial &sp, state &mystate, position_msg &pos_msg);
void Callback(const std_msgs::String::ConstPtr &sub_msg);
bool is_right(position_msg &pos_msg, int pos_choose);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seed_serial_node");
    ros::NodeHandle n; // 创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_pub", 100);
    ros::Subscriber chatter_sub = n.subscribe<std_msgs::String>("chatter_sub", 100, Callback);

    serial::Serial sp;                                            // 创建一个serial类
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10); //创建timeout
    sp.setPort("/dev/ttyUSB0");                                   //设置要打开的串口名称
    sp.setBaudrate(9600);                                         //设置串口通信的波特率
    sp.setTimeout(timeout);                                       //串口设置timeout
    try
    {
        sp.open(); //打开串口
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    if (sp.isOpen()) //判断串口是否打开成功
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    else
        return -1;

    ros::Rate sub_loop_rate(10);

    state mystate;
    order myorder;

    if (ros::ok())
    {

        myorder.move_order_init(origin);
        myorder.send(sp);
        wait(sp, mystate, 1); //运动至原点

            ROS_INFO_STREAM("Loosen claws!");
            for(int i=0;i<3;i++)
            {
                {
                    std_msgs::String message;
                    message.data = "loosen claw";
                    chatter_pub.publish(message);
                }
                sub_loop_rate.sleep();
            }
            while(true)
            {
                if(loosen_claw_state)
                    break;
                ros::spinOnce();
                sub_loop_rate.sleep();
            }                //松开爪子

            myorder.move_order_init(wp_down_msg);
            myorder.send(sp);
            wait(sp,mystate,3);//运动至工件夹取状态

            ROS_INFO_STREAM("Shrink claws!");
            for(int i=0;i<3;i++)
            {
                {
                    std_msgs::String message;
                    message.data = "shrink claw";
                    chatter_pub.publish(message);
                }
                sub_loop_rate.sleep();
            }

            while(true)
            {
                if(shrink_claw_state)
                    break;
                ros::spinOnce();
                sub_loop_rate.sleep();
            }                //收缩爪子

            ROS_INFO_STREAM("up start!");
            myorder.move_order_init(wp_up_msg);
            myorder.send(sp);
            wait(sp,mystate,2);//运动至工件上方
            ROS_INFO_STREAM("up down!");

            myorder.move_order_init(origin);
            myorder.send(sp);
            wait(sp,mystate,1);//运动至原点

            myorder.move_order_init(detect_msg);
            myorder.send(sp);
            wait(sp,mystate,4);//运动至检测工位

            for(int i=1;i<=10;i++)
            {
                detect_msg.angle6=detect_msg.angle6 + 9.0;
                myorder.move_order_init(detect_msg);
                myorder.send(sp);
                wait(sp,mystate,4);//转一个角度
                for(int j=1;j<=60;j++)
                    sub_loop_rate.sleep();

                ROS_INFO("detect %d:",i);
                {
                    std_msgs::String message;
                    message.data = "detect";
                    chatter_pub.publish(message);
                }
                while(true)
                {
                    if(detect_state)
                        break;
                    ros::spinOnce();
                    sub_loop_rate.sleep();
                }                //检测
                detect_state=false;

            }
        }

        while (ros::ok()) ;
        sp.close(); //关闭串口
        ROS_INFO("Serial Port Closed!");

        return 0;
    }

    void wait(serial::Serial & sp, state & mystate, int pos_choose)
    {
        position_msg pos_msg;
        mystate.recieve(sp, state::coordinate_value);
        get_pos(sp, mystate, pos_msg);
        while (!is_right(pos_msg, pos_choose))
        {
            // std::cout<<"error"<<std::endl;
            mystate.recieve(sp, state::coordinate_value);
            get_pos(sp, mystate, pos_msg);
            // std::cout<<mystate.pause_rt()<<std::endl;
        }
    }
    bool is_right(position_msg & pos_msg, int pos_choose)
    {
        if (pos_choose == 1)
            return (abs(pos_msg.x - origin_pos.x) + abs(pos_msg.y - origin_pos.y) + abs(pos_msg.z - origin_pos.z)) < T_;
        if (pos_choose == 2)
            return (abs(pos_msg.x - wp_up_pos.x) + abs(pos_msg.y - wp_up_pos.y) + abs(pos_msg.z - wp_up_pos.z)) < T_;
        if (pos_choose == 3)
            return (abs(pos_msg.x - wp_down_pos.x) + abs(pos_msg.y - wp_down_pos.y) + abs(pos_msg.z - wp_down_pos.z)) < T_;
        if (pos_choose == 4)
            return (abs(pos_msg.x - detect_pos.x) + abs(pos_msg.y - detect_pos.y) + abs(pos_msg.z - detect_pos.z)) < T_;
        return false;
    }

    void get_pos(serial::Serial & sp, state & mystate, position_msg & pos_msg)
    {
        mystate.recieve(sp, state::coordinate_value);
        mystate.get_position(pos_msg);
        // std::cout<<pos_msg.x<<"   "<<pos_msg.y<<"   "<<pos_msg.z<<std::endl;
    }

    void Callback(const std_msgs::String::ConstPtr &sub_msg)
    {
        std::string::size_type idx;

        if (sub_msg->data == "has loosened")
            loosen_claw_state = true;
        if (sub_msg->data == "has shrinked")
            shrink_claw_state = true;
        if (sub_msg->data.find("has detected") != std::string::npos)
        {
            detect_state = true;
            ROS_INFO_STREAM(sub_msg->data.substr(13, 6));
        }
    }
