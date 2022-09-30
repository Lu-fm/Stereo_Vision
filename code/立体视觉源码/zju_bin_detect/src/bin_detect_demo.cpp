#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <zju_bin_detect/BoxParam.h>

static ros::Publisher box_param_pub;
static zju_bin_detect::BoxParam box_param_msg;
static geometry_msgs::Pose box_pose;

void BoxPoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    // 盒子坐标
    box_pose = *msg;
    ROS_WARN("[BoxPose] = (%.2f , %.2f , %.2f)",box_pose.position.x,box_pose.position.y,box_pose.position.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bin_detect_demo");

    // 发布主题
    ros::NodeHandle n;
    box_param_pub = n.advertise<zju_bin_detect::BoxParam>("/box/param", 2);
    ros::Subscriber box_pose_sub = n.subscribe("/box/pose", 1, BoxPoseCallback);

    sleep(1);

    box_param_msg.z = 0.6;         //盒子可能的高度（单位：米）
    box_param_msg.color = 0;    //0-绿色盒子   1-黄色盒子
    box_param_pub.publish(box_param_msg);

    ros::spin();

    return 0;
}