#pragma once

// ros
#include <ros/ros.h>
#include <ros/console.h>

// serial
#include <serial/serial.h>
#include "command.h"
#define MSG_LENGTH 48
// kinematics
#include "kinematics.h"

class armController
{

public:
    armController();
    ~armController();
    void moveToCart(Eigen::Matrix<double, 4, 4> goal, double joint_speed, double claw);
    void moveToJoint(double *joints_cmd, double joint_speed, double claw);

private:
    void serialInit();
    void readMsg(joint_cmd &msg);
    void sendMsg(joint_cmd &msg);
    serial::Serial sp;
    joint_cmd jcmd = {0,0,0,0,0,0,0,0};
    uint8_t data[MSG_LENGTH];
};

void trans(numByte msg, uint8_t *data);
