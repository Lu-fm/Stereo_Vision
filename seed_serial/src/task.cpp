#include"seed_serial/arm_controller.h"
#include "seed_serial/state.h"
#define UPDATE_RATE 10
armController arm;
state armstate;
position_msg position;


bool arm_server(seed_serial::cartHorizon::Request &request,
                               seed_serial::cartHorizon::Response &response);

void stateUpdateCallback(const ros::TimerEvent &event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("arm/cmd/pose", arm_server);
    ros::Timer timer = nh.createTimer(ros::Duration(1/UPDATE_RATE), stateUpdateCallback);
    timer.start();

    Eigen::Matrix<double, 4, 4> pose;
    double theta = 0;
    double pose_euler[6] = {450, 0, 300, 0, 0, 0};
    double home[] = {-10, 30.33, 100.44, -0.5, 40.94, -83.41};
    double joint_cmd[] = {0, -3.27, 81.32, 0, 105, 0};
    double joint_speed, claw;
    double x,y,z;
    x = 300;
    y = 0;
    z = 300;
    theta = atan2(y,x);
    pose<<  0,  -sin(theta), -cos(theta), x,
            0,  cos(theta), -sin(theta), y,
            1,  0, 0, z,
            0,  0, 0, 1;  
    
    joint_speed = 5;
    claw = 1000;
    // arm.moveToJoint(home,joint_speed,claw);
    // arm.moveToCart(pose_euler, joint_speed, claw);
    // ROS_INFO_STREAM("Command has sent");
    ros::spin();
}


bool arm_server(seed_serial::cartHorizon::Request &request,
                               seed_serial::cartHorizon::Response &response)
{
    Eigen::Matrix<double, 4, 4> target;
    double theta = atan2(request.y, request.x);
    target << 0, -sin(theta), -cos(theta), request.x,
        0, cos(theta), -sin(theta), request.y,
        1, 0, 0, request.z,
        0, 0, 0, 1;
    arm.moveToCart(target, request.speed, request.claw);
    ROS_INFO_STREAM("Go to pose: \n"<< target);
    response.isSuccess = true;
    return true;
}


void stateUpdateCallback(const ros::TimerEvent &event)
{
    serial::Serial *sp;
    sp = arm.get_serial();
    
    if(sp == nullptr)
    {
        ROS_INFO("Serial Port Not Open!");
        return;
    }
    armstate.recieve(sp, state::coordinate_value);
    armstate.get_position(position);
    // position_pub.publish(position);    
    ROS_INFO("x: %d\ny:%d\nz:%d---------\n",position.x, position.y, position.z);
}
