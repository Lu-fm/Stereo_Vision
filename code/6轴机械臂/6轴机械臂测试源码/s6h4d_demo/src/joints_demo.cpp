#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joints_demo");

    // 发布主题
    ros::NodeHandle n;
    ros::Publisher arm_4_pub = n.advertise<sensor_msgs::JointState>("arm_4/joint_position", 30);
    ros::Publisher arm_5_pub = n.advertise<sensor_msgs::JointState>("arm_5/joint_position", 30);

    // 初始化消息包
    sensor_msgs::JointState joint_msg;
    joint_msg.name.resize(7);
    joint_msg.position.resize(7);
    joint_msg.velocity.resize(7); 
    joint_msg.name[0] = "joint1";
    joint_msg.name[1] = "joint2";
    joint_msg.name[2] = "joint3";
    joint_msg.name[3] = "joint4";
    joint_msg.name[4] = "joint5";
    joint_msg.name[5] = "joint_griper";
    joint_msg.name[6] = "joint_finger";
    joint_msg.position[0] = 0.0f;
    joint_msg.position[1] = 0.0f;
    joint_msg.position[2] = 0.0f;
    joint_msg.position[3] = 0.0f;
    joint_msg.position[4] = 0.0f;
    joint_msg.position[5] = 0.0f;
    joint_msg.position[6] = 0.0f; 

    // 消息包发送频率
    ros::Rate r(10);
    
    while(ros::ok())
    {
        // 发给arm4机械臂（joint1转动）
        joint_msg.position[0] += 0.1;
        arm_4_pub.publish(joint_msg);
        // 发给arm5机械臂（joint2转动）
        joint_msg.position[1] += 0.1;
        arm_5_pub.publish(joint_msg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}