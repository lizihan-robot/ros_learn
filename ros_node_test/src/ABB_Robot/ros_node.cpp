#include "ros/ros.h"
#include "ros_node.h"


void RobotController::advertiseServives(ros::NodeHandle& node_handle)
{
    getcart_server = node_handle.advertiseService("GetCartcpp", &RobotController::GetCart, this);
    ROS_INFO("Ready to GetCartesion!");
    getio_server = node_handle.advertiseService("GetIOcpp", &RobotController::GetIO, this);
    ROS_INFO("Ready to read io!");
    getjoints_server = node_handle.advertiseService("GetJointscpp", &RobotController::GetJoint, this);
    ROS_INFO("Ready to get joints!");
    setacc_server = node_handle.advertiseService("SetACCcpp", &RobotController::SetAcc, this);
    ROS_INFO("Ready to set acc!");
    setcart_server = node_handle.advertiseService("SetCartcpp", &RobotController::SetCart, this);
    ROS_INFO("Ready to SetCartesion!");
    setIO_server = node_handle.advertiseService("SetIOcpp", &RobotController::SetIO, this);
    ROS_INFO("Ready to SetIO!");
    setjoint_server = node_handle.advertiseService("SetJointscpp", &RobotController::SetJoint, this);
    ROS_INFO("Ready to SetJoints!");
    setspeed_server = node_handle.advertiseService("SetSpeedcpp", &RobotController::SetSpeed, this);
    ROS_INFO("Ready to SetSpeed!");
}

bool RobotController::GetCart(ros_node_test::GetCart::Request &req,
            ros_node_test::GetCart::Response &res)
{

    double pose[7];
    for(int i=0;i<=6;i++){
        pose[i] = i+rand()/double(RAND_MAX);
    }
    res.x = pose[0];
    res.y = pose[1];
    res.z = pose[2];
    res.qx = pose[3];
    res.qy = pose[4];
    res.qz = pose[5];
    res.qw = pose[6];
    ROS_INFO("\nCurrent pose:\n x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f",
            res.x, res.y, res.z, res.qx, res.qy, res.qz, res.qw);
    return true;
}

bool RobotController::GetIO(ros_node_test::GetIO::Request &req,
            ros_node_test::GetIO::Response &res)
{

    res.io = 1;
    ROS_INFO("port: %d \n value: %d ", req.port, res.io);
    return true;
}

bool RobotController::GetJoint(ros_node_test::GetJoint::Request &req,
            ros_node_test::GetJoint::Response &res)
{
    double joints[7];
    for(int i=0; i<7; i++)
    {
        joints[i] =i+rand()/double(RAND_MAX);
    }
    res.J1 =joints[0];
    res.J2 =joints[1];
    res.J3 =joints[2];
    res.J4 =joints[3];
    res.J5 =joints[4];
    res.J6 =joints[5];
    ROS_INFO("Current pose:\n j1:%f, j2:%f, j3:%f, j4:%f, j5:%f, j6:%f",
    res.J1, res.J2, res.J3, res.J4, res.J5, res.J6);
    return true;
}

bool RobotController::SetAcc(ros_node_test::SetAcc::Request &req,
            ros_node_test::SetAcc::Response &res)
{
    ROS_INFO("acc1: %f, acc2:%f", req.acc1, req.acc2);
    if(req.acc1<100 and req.acc2<100){
        res.status = true;
    }
    if (res.status == true)
    {
        ROS_INFO("SetACC success");
    }
    else{
        ROS_INFO("SetACC failed");
    }
    return true;
}

bool RobotController::SetCart(ros_node_test::SetCart::Request &req,
            ros_node_test::SetCart::Response &res)
{
    ROS_INFO("inut pose:\n x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f",
            req.x, req.y, req.z, req.qx, req.qy, req.qz, req.qw);
    res.x = req.x;
    res.y = req.y;
    res.z = req.z;
    res.qx = req.qx;
    res.qy = req.qy;
    res.qz = req.qz;
    res.qw = req.qw;
    ROS_INFO("Current pose:\n x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f",
            res.x, res.y, res.z, res.qx, res.qy, res.qz, res.qw);
    return true;
}


bool RobotController::SetIO(ros_node_test::SetIO::Request &req,
            ros_node_test::SetIO::Response &res)
{
    ROS_INFO("port: %d, value: %d", req.io_port, req.value);
    if(req.value != 0){
        res.status = 1;
        ROS_INFO("SetIO success");
    }
    return true;
}

bool RobotController::SetJoint(ros_node_test::SetJoint::Request &req,
            ros_node_test::SetJoint::Response &res)
{
    ROS_INFO("inut Joints:\n j1:%f, j2:%f, j3:%f, j4:%f, j5:%f, j6:%f",
            req.J1, req.J2, req.J3, req.J4, req.J5, req.J6);
    res.J1 = req.J1;
    res.J2 = req.J2;
    res.J3 = req.J3;
    res.J4 = req.J4;
    res.J5 = req.J5;
    res.J6 = req.J6;
    ROS_INFO("Current pose:\n j1:%f, j2:%f, j3:%f, j4:%f, j5:%f, j6:%f",
            res.J1, res.J2, res.J3, res.J4, res.J5, res.J6);
    return true;
}

bool RobotController::SetSpeed(ros_node_test::SetSpeed::Request &req,
            ros_node_test::SetSpeed::Response &res)
{
    ROS_INFO("inut speed:\n speed:%f", req.speed);
    if(req.speed != 0)
    {
        res.status = 1;
        ROS_INFO("setspeed success!");
    }
    return true;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle node;
    RobotController abb_node;
    abb_node.advertiseServives(node);
    ros::spin();
    return 0;
}










