#include"seed_serial/arm_controller.h"
#include"seed_serial/state.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#define UPDATE_RATE 10
armController arm;
position_msg cur_pose;
state armstate;
int idx;

bool arm_server(seed_serial::cartHorizon::Request &request,
                               seed_serial::cartHorizon::Response &response);

void updatePos(const ros::TimerEvent &event);
void pubProcess(const ros::TimerEvent &event);
void BoxPoseCB(const geometry_msgs::Pose::ConstPtr &msg);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle nh;
    ros::ServiceServer service = nh.advertiseService("arm/pose", arm_server);
    armstate.jointPub = nh.advertise<sensor_msgs::JointState>("arm/joint_pos" , 1);
    ros::Subscriber visionSub = nh.subscribe("/box/pose",10, BoxPoseCB);
   
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), updatePos);
    ros::Timer pubTimer = nh.createTimer(ros::Duration(0.1), pubProcess);
    idx = 0;
    timer.start();
    pubTimer.start();

    Eigen::Matrix<double, 4, 4> pose;
    double theta = 0;
    double pose_euler[6] = {450, 0, 300, 0, 0, 0};
    double home[] = {-10, 30.33, 100.44, -0.5, 40.94, -83.41};
    double joint_cmd[] = {0, -3.27, 81.32, 0, 105, 0};
    double joint_speed, claw;
    double x,y,z;
    x = 400;
    y = 0;
    z = 250;
    theta = atan2(y,x);
    pose<<  0,  -sin(theta), -cos(theta), x,
            0,  cos(theta), -sin(theta), y,
            1,  0, 0, z,
            0,  0, 0, 1;  
    
    joint_speed = 5;
    claw = 1000;
    arm.moveToCart(pose, joint_speed, claw);
    // ROS_INFO_STREAM("Command has sent");
    ros::spin();
}
// current x:3722
// current y:0
// current z:3487-------



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

void updatePos(const ros::TimerEvent &event)
{
    // ROS_INFO("OK");
    bool isReceived = armstate.recieve(*arm.get_serial(), state::coordinate_value);
    if(isReceived)
    {
        // for(int i = 0; i < 6; i++)
        // {
        //     std::cout<<"joint "<<i<<" :"<<armstate.joint_pos[i]<<std::endl;
        //     if(i==5)
        //         std::cout<<"---------"<<idx<<"--------\n";
                
        // }
        // idx++;
        // std::cout<<"current x:"<<cur_pose.x<<"\n"<<"current y:"<<cur_pose.y<<"\n"<<"current z:"<<cur_pose.z<<"-------\n";    
    }
        
}

void pubProcess(const ros::TimerEvent &event)
{
    sensor_msgs::JointState msg;
    if(armstate.jointFlag == 2)
    {
        for(int i = 0; i < 6; i++)
            msg.position.push_back(armstate.joint_pos[i]);
        armstate.jointPub.publish(msg);
    }
}

void BoxPoseCB(const geometry_msgs::Pose::ConstPtr &msg)
{
    armstate.box_pose[0] = msg->position.x;
    armstate.box_pose[1] = msg->position.y;
    armstate.box_pose[2] = msg->position.z;
    armstate.box_pose[3] = msg->orientation.x;
    armstate.box_pose[4] = msg->orientation.y;
    armstate.box_pose[5] = msg->orientation.z;
    armstate.box_pose[6] = msg->orientation.w;
    
}
