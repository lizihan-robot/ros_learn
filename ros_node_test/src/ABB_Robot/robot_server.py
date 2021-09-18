#! /usr/bin/python
import rospy
from ros_node_test.srv import GetCart, GetCartResponse, GetIO, GetIOResponse, GetJoint, GetJointResponse, SetAcc, SetAccResponse, SetCart, SetCartResponse, SetIO, SetIOResponse, SetJoint, SetJointResponse, SetSpeed, SetSpeedResponse


class RobotNodeServer(object):
    def get_cart(self, req):
        return GetCartResponse(1,2,3,4,5,6,7)

    def get_io(self, req):
        return GetIOResponse(1)

    def get_joint(self, req):
        return GetJointResponse(1.1, 2.2, 3.3, 4.4, 5.5, 6.6)

    def setacc(self, req):
        print("acc1: {} \nacc2: {}".format(req.acc1,req.acc2))
        return SetAccResponse(1)

    def set_cart(self, req):
        print("x: {} \ny: {} \nz: {} \nqx: {} \nqy: {} \nqz: {} \nqw: {}".format(req.x, req.y, req.z, req.qx, req.qy, req.qz, req.qw))
        return SetCartResponse(req.x, req.y, req.z, req.qx, req.qy, req.qz, req.qw)

    def set_io(self, req):
        print("io_port:",req.io_port)
        return SetIOResponse(req.io_port)

    def set_joints(self, req):
        print("j1: {} \nj2: {} \nj3: {} \nj4: {} \nj5: {} \nj6: {} ".format(req.J1, req.J2, req.J3, req.J4, req.J5, req.J6))
        return SetJointResponse(req.J1, req.J2, req.J3, req.J4, req.J5, req.J6)

    def set_speed(self, req):
        print("set_speed:{}".format(req.speed))
        return SetSpeedResponse(1)

    def robot_control_node(self):
        get_cart_server = rospy.Service('GetCart',GetCart, self.get_cart)
        print("get robot cartesion")

        get_io_server = rospy.Service('GetIO',GetIO, self.get_io)
        print("get io")

        get_joint_server = rospy.Service('GetJoint',GetJoint, self.get_joint)
        print("get robot joints")

        set_acc_server = rospy.Service('SetAcc',SetAcc, self.setacc)
        print("set robot acc")

        set_cart_server = rospy.Service('SetCart',SetCart, self.set_cart)
        print("set robot cartesion")

        set_io_server = rospy.Service('SetIO',SetIO, self.set_io)
        print("set robot IO")
        rospy.spin()

        set_joint_server = rospy.Service('SetJoint',SetJoint, self.set_joints)
        print("set robot joints")

        set_speed_server = rospy.Service('SetSpeed',SetSpeed, self.set_speed)
        print("set robot speed")
        

if __name__ == "__main__":
    rospy.init_node('robot_server')
    rob = RobotNodeServer()
    rob.robot_control_node()
    rospy.spin()