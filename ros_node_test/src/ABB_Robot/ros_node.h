#include <ros/ros.h>
#include <iostream>
#include <ros_node_test/GetCart.h>
#include <ros_node_test/GetIO.h>
#include <ros_node_test/GetIORequest.h>
#include <ros_node_test/GetJoint.h>
#include <ros_node_test/SetAcc.h>
#include <ros_node_test/SetCart.h>
#include <ros_node_test/SetIO.h>
#include <ros_node_test/SetJoint.h>
#include <ros_node_test/SetSpeed.h>

#include <stdlib.h>
#include <time.h>
using namespace std;

class RobotController{
    public:
    ros::ServiceServer getcart_server;
    ros::ServiceServer getio_server;
    ros::ServiceServer getjoints_server;
    ros::ServiceServer setacc_server;
    ros::ServiceServer setcart_server;
    ros::ServiceServer setIO_server;
    ros::ServiceServer setjoint_server;
    ros::ServiceServer setspeed_server;

    void advertiseServives(ros::NodeHandle& node_handle);

    bool GetCart(ros_node_test::GetCart::Request &req, ros_node_test::GetCart::Response &res);

    bool GetIO(ros_node_test::GetIO::Request &req, ros_node_test::GetIO::Response &res);

    bool GetJoint(ros_node_test::GetJoint::Request &req, ros_node_test::GetJoint::Response &res);

    bool SetAcc(ros_node_test::SetAcc::Request &req, ros_node_test::SetAcc::Response &res);

    bool SetCart(ros_node_test::SetCart::Request &req, ros_node_test::SetCart::Response &res);

    bool SetIO(ros_node_test::SetIO::Request &req, ros_node_test::SetIO::Response &res);

    bool SetJoint(ros_node_test::SetJoint::Request &req,ros_node_test::SetJoint::Response &res);

    bool SetSpeed(ros_node_test::SetSpeed::Request &req, ros_node_test::SetSpeed::Response &res);

