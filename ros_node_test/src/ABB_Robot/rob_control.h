#include <iostream>
#include <ros/ros.h>
#include <cstdlib>
#include <array>
#include "ros_node_test/GetCart.h"
#include "ros_node_test/GetIO.h"
#include "ros_node_test/GetJoint.h"
#include "ros_node_test/SetAcc.h"
#include "ros_node_test/SetCart.h"
#include "ros_node_test/SetIO.h"
#include "ros_node_test/SetJoint.h"
#include "ros_node_test/SetSpeed.h"


using cartesion = std::array<double, 7>;   
using joints = std::array<double, 6>;  
ros::ServiceClient getcart_;
ros::ServiceClient getio_;
ros::ServiceClient getjoint_;
ros::ServiceClient setacc_;
ros::ServiceClient setcart_;
ros::ServiceClient setio_;
ros::ServiceClient setjoint_;
ros::ServiceClient setspeed_;


ros_node_test::GetCart get_cart_srv;
ros_node_test::GetIO get_io_srv;
ros_node_test::GetJoint get_joint_srv;
ros_node_test::SetAcc set_acc_srv;
ros_node_test::SetCart set_cart_srv;
ros_node_test::SetIO set_io_srv;
ros_node_test::SetJoint set_joints_srv;
ros_node_test::SetSpeed set_speed_srv;



class RobClient{
    public:


    cartesion GetCart();
    bool GetIO(int port);
    joints GetJoint();
    bool SetAcc(double acc1, double acc2);
    cartesion SetCart(double cart[]);
    bool SetIO(int io_port, int value);
    joints SetJoints(double joint[]);
    bool SetSpeed(float speedv);


};

