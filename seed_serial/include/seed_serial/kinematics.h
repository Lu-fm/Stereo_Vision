#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Dense>

class Kinematics
{

private:
    Eigen::Matrix<double, 4, 4> T0_1, T1_2, T2_3, T3_4, T4_5, T5_6, T6_eef, Teef_6, T0_6, T0_eef;
    Eigen::Matrix<double, 6, 1> angles;
    double joint_num;
    double Rs, rs, A;
    double x, y, z, roll, yaw, pitch;
    double a1, a2, a3;
    double d2, d4, eef_offset;

public: 
    Kinematics();
    ~Kinematics();

    void set_cur_angle(Eigen::Matrix<double, 6, 1> angles);
    void do_forward_kin(Eigen::Matrix<double, 6, 1> angles, Eigen::Matrix<double, 4, 4> &eefInBase);
    bool do_inverse_kin(Eigen::Matrix<double, 4, 4> eef, Eigen::Matrix<double, 6, 1> &solu);
};

void eulerZYX2matrix(Eigen::Matrix<double, 3, 1> euler, Eigen::Matrix<double, 4, 4> &rot);
void matrix2eulerZYX(Eigen::Matrix<double, 4, 4> rot, Eigen::Matrix<double, 3, 1> &eulerZYX);
