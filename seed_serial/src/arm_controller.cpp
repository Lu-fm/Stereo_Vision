#include "seed_serial/arm_controller.h"
#include "seed_serial/state.h"

armController::armController()
{
    serialInit();
};

armController::~armController()
{
    sp->close();
};

serial::Serial * armController::get_serial()
{
    if(sp.isOpen())
        return &sp;
    else
        ROS_INFO("Serial Port is NOT Open");
}


void armController::serialInit()
{
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10); //创建timeout
    sp->setPort("/dev/ttyUSB0");                                   //设置要打开的串口名称
    sp->setBaudrate(9600);                                         //设置串口通信的波特率
    sp->setTimeout(timeout);
    try
    {
        sp->open(); //打开串口
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
    }
    if (sp->isOpen()) //判断串口是否打开成功
        ROS_INFO_STREAM("The serial port is opened.");
    else
        ROS_INFO_STREAM("The serial port is not opened.");
};

void armController::readMsg(joint_cmd &msg){
    // to do
};

void armController::sendMsg(joint_cmd &msg)
{
    data[0] = 0xee;
    data[1] = '3';
    data[2] = 1;
    trans(msg.angle1, &(data[3]));
    trans(msg.angle2, &(data[7]));
    trans(msg.angle3, &(data[11]));
    trans(msg.angle4, &(data[15]));
    trans(msg.angle5, &(data[19]));
    trans(msg.angle6, &(data[23]));
    trans(msg.claw, &(data[27]));
    trans(bnf32(0.0), &(data[39]));
    trans(msg.speed, &(data[43]));
    data[47] = 0xef;
    size_t length = sp->write(data, MSG_LENGTH);
};

void armController::moveToCart(Eigen::Matrix<double, 4, 4> goal, double joint_speed, double claw)
{
    Eigen::Matrix<double, 6, 1> solu;
    ROS_INFO_STREAM("Target:\n"
                    << goal);
    bool has_solu = arm_kin.do_inverse_kin(goal, solu);
    if (!has_solu)
    {
        ROS_WARN("Inverse Solution NOT FOUND!\n");
        return;
    }
    double solu_deg[6];
    // ROS_INFO_STREAM("The solution:\n");
    for (int i = 0; i < 6; i++)
    {
        solu_deg[i] = solu[i] * 180 / M_PI;
        std::cout << solu_deg[i] << std::endl;
    }
    moveToJoint(solu_deg, joint_speed, claw);
};

void armController::moveToCart(double *pose, double joint_speed, double claw)
{
    Eigen::Matrix<double, 4, 4> target;
    Eigen::Matrix<double, 3, 1> euler;
    Eigen::Matrix<double, 6, 1> solu;
    double solu_deg[6];

    euler << pose[3], pose[4], pose[5];
    eulerZYX2matrix(euler, target);
    target.block<3, 1>(0, 3) << pose[0], pose[1], pose[2];
    ROS_INFO_STREAM("euler:\n"
                    << euler);
    ROS_INFO_STREAM("Target:\n"
                    << target);
    arm_kin.do_inverse_kin(target, solu);
    for (int i = 0; i < 6; i++)
    {
        solu_deg[i] = solu[i] * 180 / M_PI;
        std::cout << "Solu Deg: " << solu_deg[i] << "\n";
    }
    if (abs(solu_deg[3]) > 3)
    {
        ROS_WARN("Singularity occurs, plz choose another pose");
        return;
    }
    moveToJoint(solu_deg, joint_speed, claw);
};

void armController::moveToJoint(double *joint_angle, double joint_speed, double claw)
{
    // if (sizeof(joint_angle) / sizeof(joint_angle[0]) != 6)
    // {
    //     ROS_INFO_STREAM("Input angle number is "<<sizeof(joint_angle) / sizeof(joint_angle[0])<<", while is shold be 6");
    //     return;
    // }
    if ((claw - 500) * (claw - 2500) > 0)
    {
        ROS_INFO_STREAM("Claw should be 500~2500");
        claw = 1500;
    }

    joint_speed = joint_speed * (joint_speed - 10) < 0 ? joint_speed : 10;

    jcmd.angle1 = joint_angle[0];
    jcmd.angle2 = joint_angle[1];
    jcmd.angle3 = joint_angle[2];
    jcmd.angle4 = joint_angle[3];
    jcmd.angle5 = joint_angle[4];
    jcmd.angle6 = joint_angle[5];
    jcmd.speed = joint_speed;
    jcmd.claw = claw;
    sendMsg(jcmd);
};

void armController::home(){

};

void armController::forward_kin(double *angles, Eigen::Matrix<double, 4, 4> &pose)
{
    Eigen::Matrix<double, 6, 1> joint_angles;
    for (int i = 0; i < 6; i++)
        joint_angles[i] = angles[i];
    arm_kin.set_cur_angle(joint_angles);
    arm_kin.do_forward_kin(joint_angles, pose);
};

// END OF CLASS ARMCONTROLLER

void trans(bnf32 msg, uint8_t *data)
{
    for (int i = 0; i < 4; i++)
        data[i] = msg.byte[i];
};

