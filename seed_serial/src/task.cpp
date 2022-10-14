#include"seed_serial/armController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle n; // 
    armController arm;
    Eigen::Matrix<double, 4, 4> pose;
    double pose_euler[6] = {400, 0, 400, 0, -40, 0};
    double home[] = {-10, 30.33, 100.44, -0.5, 40.94, -83.41};
    double joint_cmd[] = {0, -3.27, 81.32, 0, 105, 0};
    double joint_speed, claw;
    pose<< 0, 1, 0, 345,
          -1, 0, 0, -80,
           0, 0, 1, 400,
           0, 0, 0, 1;  
    
    joint_speed = 5;
    claw = 1000;
    // arm.moveToJoint(joint_cmd,joint_speed,claw);
    arm.moveToCart(pose, joint_speed, claw);
    // for(int i = 0; i < 20; i++)
    // {
    //    pose_euler[4] = -60 - i;
    //    arm.moveToCart(pose_euler,joint_speed,claw);
    //    std::cout<<"----------------\n";
    // }
    ROS_INFO_STREAM("Command has sent");
    ROS_INFO_STREAM("Curernt pose: \n"<<pose);
}