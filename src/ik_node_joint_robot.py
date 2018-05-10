#!/usr/bin/env python

import rospy, math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64


class IK():

    def callback(self, data):
        l1 = 200
        l2 = 200
        x = -data.y
        y = data.z
        if y < l1 + l2:
            d = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
            beta = math.atan2(math.sqrt(1 - d**2), d)
            k1 = l1 + l2 * math.cos(beta)
            k2 = l2 * math.sin(beta)
            lamda = math.atan2(k2, k1)
            alpha = math.atan2(y,x) - lamda
            self.alpha = math.pi/2 -alpha
            self.beta = -beta
            self.shoulder.publish(self.alpha)
            self.elbow.publish(self.beta)
        else:
            print "out of arm range!!!"

    def callback2 (self, data):

        pitch = data.x
        roll = -data.y
        gripper = data.z
        self.pitch.publish(-1.57 + pitch + (-self.alpha - self.beta))
        self.roll.publish(roll)
        self.grip.publish(gripper*2)


    def __init__(self):
        rospy.init_node("inverse_kinematics")
        rospy.Subscriber("position", Vector3, self.callback)
        rospy.Subscriber("orientation", Vector3, self.callback2)
        self.alpha = 0.0
        self.beta = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.shoulder = rospy.Publisher("hek/shoulder_joint_position_controller/command", Float64, queue_size=10)
        self.elbow = rospy.Publisher("hek/elbow_joint_position_controller/command", Float64, queue_size=10)
        self.pitch = rospy.Publisher("hek/wrist_pitch_joint_position_controller/command", Float64, queue_size=10)
        self.roll = rospy.Publisher("hek/wrist_rot_joint_position_controller/command", Float64, queue_size=10)
        self.grip = rospy.Publisher("hek/gripper_joint_position_controller/command", Float64, queue_size=10)
        print "IK Ready"




        rospy.spin()

if __name__ == "__main__":
    inverse_kinematic = IK()
