/**
 * @file arm_controller.h
 * @author Fangmin Lu (lufangmin@zju.edu.cn)
 * @brief This is the class of arm controller.
 * @version 0.1
 * @date 2022-10-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

// ros
#include <ros/ros.h>
#include <ros/console.h>
#include "seed_serial/cartHorizon.h"

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
    void moveToCart(double *pose, double joint_speed, double claw);
    void moveToJoint(double *joints_cmd, double joint_speed, double claw);
    void forward_kin(double *angles, Eigen::Matrix<double, 4, 4> &pose);
    void home();
    serial::Serial *get_serial();

private:
    void serialInit();
    void readMsg(joint_cmd &msg);
    void sendMsg(joint_cmd &msg);
    serial::Serial *sp;
    Kinematics arm_kin;
    joint_cmd jcmd = {0,0,0,0,0,0,0,0};
    uint8_t data[MSG_LENGTH];


};

void trans(bnf32 msg, uint8_t *data);
