#! /usr/bin/python
import rospy
from ros_node_test.srv import *

class Robot():
    def get_cart_client(self):
        rospy.wait_for_service('GetCart')
        try:
            rob_control = rospy.ServiceProxy('GetCart',GetCart)
            pose = rob_control()
            pose_list = [pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]
            print("robot pose:{}".format(pose_list))
        except rospy.ServiceException as e:
            print(e)

    def get_io(self):
        rospy.wait_for_service('GetIO')
        try:
            rob_control = rospy.ServiceProxy('GetIO',GetIO)
            io = rob_control()
            print("robot io:{}".format(io))
        except rospy.ServiceException as e:
            print(e)

    def set_cart(self, x, y, z, qx, qy, qz, qw):
        rospy.wait_for_service('SetCart')
        try:
            rob_control = rospy.ServiceProxy('SetCart',SetCart)
            pose = rob_control(x, y, z, qx, qy, qz, qw)
            pose_list = [pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]
            print("robot pose:{}".format(pose_list))
        except rospy.ServiceException as e:
            print(e)

if __name__ == "__main__":
    rob = Robot()
    # import pdb;pdb.set_trace()
    rob.get_cart_client()
    # rob.get_io()
    # rob.set_cart(1,2,3,5,6,7,8)