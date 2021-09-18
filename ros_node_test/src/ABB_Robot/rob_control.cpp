#include <rob_control.h>


cartesion RobClient::GetCart()
{

}

bool RobClient::GetIO(int port)
{

}

joints RobClient::GetJoint()
{

}

bool RobClient::SetAcc(double acc1, double acc2)
{

}

cartesion RobClient::SetCart(double cart[])
{

}

bool RobClient::SetIO(int io_port, int value)
{

}

joints RobClient::SetJoints(double joint[])
{

}

bool RobClient::SetSpeed(float speedv)
{

}

int main(int argc, char **argv)
{
    RobClient rob;
    ros::init(argc, argv, "robot_cmd");
    ros::NodeHandle node;
    getcart_ = node.serviceClient<ros_node_test::GetCart>("GetCartcpp");
    getio_ = node.serviceClient<ros_node_test::GetIO>("GetIOcpp");
    getjoint_ = node.serviceClient<ros_node_test::GetJoint>("GetJointscpp");
    setacc_ = node.serviceClient<ros_node_test::SetAcc>("SetACCcpp");
    setcart_ = node.serviceClient<ros_node_test::SetCart>("SetCartcpp");
    setio_ = node.serviceClient<ros_node_test::SetIO>("SetIOcpp");
    setjoint_ = node.serviceClient<ros_node_test::SetJoint>("SetJointscpp");
    setspeed_ = node.serviceClient<ros_node_test::SetSpeed>("SetSpeedcpp");
    ros_node_test::GetCart get_cart_srv;



}