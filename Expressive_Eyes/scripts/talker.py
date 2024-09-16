#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from pynput.keyboard import Key, Listener
from pynput import keyboard


class JointStatePublisher():
    """
    A class that prints the positions of desired joints in Stretch.
    """
    def __init__(self, joints):
        """
        Function that initializes the subscriber.
        :param self: The self reference.
        """
        self.sub = rospy.Subscriber('joint_states', JointState, self.callback)

        self.pub = rospy.Publisher('head_camera_jointstate', Float64MultiArray, queue_size=10)

        self.head_poses = Float64MultiArray()

        self.joints = joints

        # self.key = key


    def callback(self, msg):
        """
        Callback function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param msg: The JointState message.
        """
        self.joint_states = msg
        joint_positions = []
        for joint in self.joints:
            if joint == "wrist_extension":
                index = self.joint_states.name.index('joint_arm_l0')
                joint_positions.append(4*self.joint_states.position[index])
                continue
            index = self.joint_states.name.index(joint)
            joint_positions.append(self.joint_states.position[index])
            # joint_positions.append(format(key))
            # print('stuff printing', joint_positions)
        
        self.head_poses.data = joint_positions
        
        self.pub.publish(self.head_poses)



    def print_states(self, joints):
        """
        print_states function to deal with the incoming JointState messages.
        :param self: The self reference.
        :param joints: A list of string values of joint names.
        """
        joint_positions = []
        for joint in joints:
            if joint == "wrist_extension":
                index = self.joint_states.name.index('joint_arm_l0')
                joint_positions.append(4*self.joint_states.position[index])
                continue
            index = self.joint_states.name.index(joint)
            joint_positions.append(self.joint_states.position[index])
        print("name: " + str(joints))
        print("position: " + str(joint_positions))
        rospy.signal_shutdown("done")
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('joint_state_printer', anonymous=True)

    joints = ["joint_head_pan", "joint_head_tilt"]
    JSP = JointStatePublisher(joints)
    rospy.sleep(.1)
    #, "wrist_extension", "joint_wrist_yaw"]
    #joints = ["joint_head_pan","joint_head_tilt", joint_gripper_finger_left", "joint_gripper_finger_right"]
    # JSP.print_states(joints)
    rospy.spin()
