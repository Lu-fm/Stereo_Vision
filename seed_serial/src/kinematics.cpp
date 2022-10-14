#include "seed_serial/kinematics.h"

Kinematics::Kinematics()
{
    joint_num = 6;
    a1 = 53;
    d2 = 255 - 60;
    a2 = 275;
    a3 = 93.8;
    d4 = -258.5;
    eef_offset = 99; // We can set it later

    angles.setZero();
    T0_1.setIdentity();
    T1_2.setIdentity();
    T2_3.setIdentity();
    T3_4.setIdentity();
    T4_5.setIdentity();
    T5_6.setIdentity();
    T6_eef.setIdentity();

    T0_1.block<3, 1>(0, 3) << 0, 0, 0; // 112, 0, 255
    T1_2.block<3, 1>(0, 3) << 53, 0, 255 - 60;
    T2_3.block<3, 1>(0, 3) << 275, 0, 0;
    T3_4.block<3, 1>(0, 3) << 93.8, -258.5, 0;
    T4_5.block<3, 1>(0, 3) << 0, 0, 0;
    T5_6.block<3, 1>(0, 3) << 0, 0, 0;
    T6_eef.block<3, 1>(0, 3) << 0, 0, -eef_offset;
    std::cout << "Initialize Forward Kinematics Done!" << std::endl;
}

Kinematics::~Kinematics()
{
}

void Kinematics::set_cur_angle(Eigen::Matrix<double, 6, 1> joint_angles)
{
    angles = joint_angles * M_PI / 180;

    T0_1.block<3, 3>(0, 0) << cos(angles[0]), -sin(angles[0]), 0,
        sin(angles[0]), cos(angles[0]), 0,
        0, 0, 1;

    T1_2.block<3, 3>(0, 0) << sin(angles[1]), cos(angles[1]), 0,
        0, 0, 1,
        cos(angles[1]), -sin(angles[1]), 0;

    T2_3.block<3, 3>(0, 0) << cos(angles[2]), -sin(angles[2]), 0,
        -sin(angles[2]), -cos(angles[2]), 0,
        0, 0, -1;

    T3_4.block<3, 3>(0, 0) << cos(angles[3]), -sin(angles[3]), 0,
        0, 0, -1,
        sin(angles[3]), cos(angles[3]), 0;

    T4_5.block<3, 3>(0, 0) << sin(angles[4]), cos(angles[4]), 0,
        0, 0, 1,
        cos(angles[4]), -sin(angles[4]), 0;

    T5_6.block<3, 3>(0, 0) << 0, 0, -1,
        cos(angles[5]), -sin(angles[5]), 0,
        -sin(angles[5]), -cos(angles[5]), 0;
}

void Kinematics::do_forward_kin(Eigen::Matrix<double, 6, 1> joint_angles, Eigen::Matrix<double, 4, 4> &eefInBase)
{
    joint_angles[2] -= 110;
    joint_angles[4] -= 90;
    set_cur_angle(joint_angles);
    T0_eef = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_eef;
    eefInBase = T0_eef;

#ifdef ANASOL
    std::cout << "T0_4: \n"
              << T0_1 * T1_2 * T2_3 * T3_4 << std::endl;
    double x, y, z;
    x = cos(angles[0]) * sin(angles[1] - angles[2]) * a3 - cos(angles[0]) * cos(angles[1] - angles[2]) * d4 + cos(angles[0]) * sin(angles[1]) * a2 + cos(angles[0]) * a1;
    y = x * tan(angles[0]);
    z = cos(angles[1] - angles[2]) * a3 + sin(angles[1] - angles[2]) * d4 + cos(angles[1]) * a2 + d2;
#endif
}

bool Kinematics::do_inverse_kin(Eigen::Matrix<double, 4, 4> eef, Eigen::Matrix<double, 6, 1> &solu)
{
    double m, n, f;
    Eigen::Matrix<double, 4, 4> T0_6, T4_6;
    Eigen::Matrix<double, 6, 1> solud;

    solu.setZero();
    T0_6 = eef * T6_eef.inverse();
    x = T0_6(0, 3);
    y = T0_6(1, 3);
    z = T0_6(2, 3);

    Rs = pow(x, 2) + pow(y, 2) + pow(z, 2);
    rs = Rs - pow(z, 2);
    A = (pow(a3, 2) + pow(a2, 2) + pow(a1, 2) + pow(d4, 2) + pow(d2, 2) - Rs) / 2 + a1 * (sqrt(rs) - a1) + d2 * (z - d2);

    m = d4 * a2;
    n = -a2 * a3;
    f = A;

    solu[2] = 2 * atan((m + sqrt(m * m + n * n - f * f)) / (f + n)); // m+ or m-

    m = d4 - sin(solu[2]) * a2;
    n = a3 + cos(solu[2]) * a2;
    f = z - d2;
    solu[1] = solu[2] + 2 * atan((m + sqrt(m * m + n * n - f * f)) / (f + n)); // m+ or m-

    if (rs != 0)
    {
        solu[0] = atan2(y, x);
    }

    // solu[2] += 110 * M_PI / 180; // specially for the robot

    solud = solu * 180 / M_PI;
    set_cur_angle(solud);

    T4_6 = (T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6).inverse() * T0_6;
    std::cout << "T4_6:\n"
              << T4_6 << std::endl;

    solu[4] = atan2(sqrt(pow(T4_6(2, 0), 2) + pow(T4_6(2, 1), 2)), T4_6(2, 2));
    if (abs(solu[4] - M_PI) < 0.01)
    {
        solu[3] = 0;
        solu[5] = atan2(T4_6(0, 1), -T4_6(0, 0));
    }
    else if (abs(solu[4]) < 0.01)
    {
        solu[3] = 0;
        solu[5] = atan2(-T4_6(0, 1), T4_6(0, 0));
    }
    else
    {
        solu[3] = atan2(T4_6(1, 2) / sin(solu[4]), T4_6(0, 2) / sin(solu[4]));
        solu[5] = atan2(T4_6(2, 1) / sin(solu[4]), -T4_6(2, 0) / sin(solu[4]));
    }

    solu[3] = -solu[3];
    solu[4] = -solu[4];

    solu[4] += M_PI / 2; // specially for our robot
    solu[2] += 110 * M_PI / 180;
    for (int i = 0; i < solu.size(); i++)
    {
        if (isnan(solu[i]))
        {
            std::cout << "out of workspace!" << std::endl;
            return false;
        }
        if (solu[i] > M_PI)
            solu[i] = 2 * M_PI - solu[i];
    }
    return true;
}

void eulerZYX2matrix(Eigen::Matrix<double, 3, 1> euler, Eigen::Matrix<double, 4, 4> &rot)
{
    double a, b, g;
    euler *= M_PI / 180;
    g = euler[0]; // roll
    b = euler[1]; // pitch
    a = euler[2]; // yaw
    rot.setIdentity();
    rot.block<3, 3>(0, 0) << cos(a) * cos(b), cos(a) * sin(b) * sin(g) - sin(a) * cos(g), cos(a) * sin(b) * cos(g) + sin(a) * sin(g),
        sin(a) * cos(b), sin(a) * sin(b) * sin(g) + cos(a) * cos(g), sin(a) * sin(b) * cos(g) - cos(a) * sin(g),
        -sin(b), cos(b) * sin(g), cos(b) * cos(g);
    // for (int i = 0; i < rot.rows(); i++)
    // {
    //     for (int j = 0; j < rot.cols(); j++)
    //         if (abs(rot(i, j)) < EPSILON)
    //             rot(i, j) = 0;
    // }
}

void matrix2eulerZYX(Eigen::Matrix<double, 4, 4> rot, Eigen::Matrix<double, 3, 1> &eulerZYX)
{
    if (rot(0, 0) == 0 && rot(1, 0) == 0)
    {
        eulerZYX[1] = M_PI / 2;
        eulerZYX[2] = 0;
        eulerZYX[0] = atan2(rot(0, 1), rot(1, 1));
    }
    else
    {
        eulerZYX[1] = atan2(-rot(2, 0), sqrt(pow(rot(0, 0), 2) + pow(rot(1, 0), 2)));
        eulerZYX[2] = atan2(rot(1, 0), rot(0, 0));
        eulerZYX[0] = atan2(rot(2, 1), rot(2, 2));
    }
    eulerZYX *= 180 / M_PI;
}

int main(int argc, char **argv)
{
    // Note: The joint angles will have an offset :[0,0,110,0,90,0] for our robot.
    Kinematics arm;
    Eigen::Matrix<double, 4, 4> eef, eef_solu;
    Eigen::Matrix<double, 6, 1> solu;
    Eigen::Matrix<double, 6, 1> joints, home_joints;
    std::vector<double> theta = {5, 10, 15, 3, 8, 13, 0};
    Eigen::Matrix<double, 3, 1> euler, euler_solu;
    Eigen::Matrix<double, 4, 4> home;
    home.setIdentity();
    home.block<3, 1>(0, 3) << 325, 0, 421;
    joints << 5, 74.5, 96.0, 0.0, 90.0, 90.0;
    home_joints << 0, -18, 79, 0, 30, 0; // home
    // Board Origin
    // eulerZ(90)Y(0)X(180), pos:277, -280, 25
    eef << 0.5, -sqrt(3) / 2, 0, 400,
        sqrt(3) / 2, 0.5, 0, 50,
        0, 0, 1, -20,
        0, 0, 0, 1;

    euler << 0,0,60;
    eulerZYX2matrix(euler, eef);
    eef.block<3,1>(0,3) << 300, 50, -20;

    // arm.do_forward_kin(home_joints * 180 / M_PI, home);
    // std::cout << "home:\n"
    //           << home << std::endl
    //           << "------" << std::endl;

    arm.do_inverse_kin(eef, solu);
    std::cout << "solu: \n"
              << solu * 180 / M_PI << std::endl
              << "------" << std::endl;

    std::cout << "eef:\n"
              << eef << std::endl
              << "------" << std::endl;
    solu[3] = 0;
    solu[5] = euler[2]*M_PI/180 - solu[0];
    arm.do_forward_kin(solu * 180 / M_PI, eef_solu);

    std::cout << "eef_solu:\n"
              << eef_solu << std::endl
              << "------" << std::endl;
}

// 480， 58， -23
// "{joint1: 6.89, joint2: 69.75, joint3: 108.66, joint4: 0.0, joint5: 71.10, joint6: -6.89, speed: 3.0,

// 434, 86, 18