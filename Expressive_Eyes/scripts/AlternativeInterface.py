#!/usr/bin/env python3
import typing
import rospy
import sys
import os
import cv2
import logging
import time
import shapely
import actionlib
import datetime
import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QMainWindow, QStackedWidget
from PyQt5.QtCore import Qt

class CameraController():
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.tilt_up
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.tilt_down
            },
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.turn_left
            },
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.turn_right
            }
        ]
        self.delt_vert = 0.125
        self.delt_horiz = 0.125
        self.parent = parent
        self.joint_states = None
        self.head_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        server_reached = self.head_client.wait_for_server(timeout=rospy.Duration(60.0))
        self.joints_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        #self.head_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)

    def joint_states_cb(self, data):
        self.joint_states = data

    def mouseover_event(self, event):
        pass

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def tilt_up(self):
        command = {'joint': 'joint_head_tilt', 'delta': self.delt_vert}
        self.send_command(command)

    def tilt_down(self):
        command = {'joint': 'joint_head_tilt', 'delta': -self.delt_vert}
        self.send_command(command)

    def turn_left(self):
        command = {'joint': 'joint_head_pan', 'delta': self.delt_horiz}
        self.send_command(command)

    def turn_right(self):
        command = {'joint': 'joint_head_pan', 'delta': -self.delt_horiz}
        self.send_command(command)

    def send_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.head_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)
        

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")

class NavigationController():
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.go_forwards,
                "mouseover": self.mouseover_forwards
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.go_backwards,
                "mouseover": self.mouseover_backwards
            },
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.turn_left,
                "mouseover": self.mouseover_left
            },
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.turn_right,
                "mouseover": self.mouseover_right
            }
        ]
        self.parent = parent
        self.move_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)

    def mouseover_event(self, event):
        return
        """dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["mouseover"]()"""

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def go_forwards(self):
        msg = Twist()
        msg.linear.x = 2
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.linear.x = 0
        self.move_publisher.publish(msg)

    def go_backwards(self):
        msg = Twist()
        msg.linear.x = -2
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.linear.x = 0
        self.move_publisher.publish(msg)
    
    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 2
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.angular.z = 0
        self.move_publisher.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = -2
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.angular.z = 0
        self.move_publisher.publish(msg)

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")

class MainCameraController():
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.tilt_up
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.tilt_down
            },
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.turn_left
            },
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.turn_right
            }
        ]
        self.delt_vert = 0.125
        self.delt_horiz = 0.125
        self.parent = parent
        self.joint_states = None

        #For real
        #self.head_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        #For simulation
        self.head_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        server_reached = self.head_client.wait_for_server(timeout=rospy.Duration(60.0))
        self.joints_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        #self.head_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)

    def joint_states_cb(self, data):
        self.joint_states = data

    def mouseover_event(self, event):
        pass

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def tilt_up(self):
        command = {'joint': 'joint_head_tilt', 'delta': self.delt_vert}
        self.send_command(command)

    def tilt_down(self):
        command = {'joint': 'joint_head_tilt', 'delta': -self.delt_vert}
        self.send_command(command)

    def turn_left(self):
        command = {'joint': 'joint_head_pan', 'delta': self.delt_horiz}
        self.send_command(command)

    def turn_right(self):
        command = {'joint': 'joint_head_pan', 'delta': -self.delt_horiz}
        self.send_command(command)

    def send_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.head_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)
        

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")

class ArmController():
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.move_up
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.move_down
            },
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.extend
            },
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.retract
            }
        ]
        self.delt_vert = 0.05
        self.delt_horiz = 0.05
        self.parent = parent
        self.joint_states = None

        #For real
        #self.arm_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        #For simulation
        self.arm_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        server_reached = self.arm_client.wait_for_server(timeout=rospy.Duration(60.0))
        self.joints_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        #self.head_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)

    def joint_states_cb(self, data):
        self.joint_states = data

    def mouseover_event(self, event):
        pass

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def move_up(self):
        command = {'joint': 'joint_lift', 'delta': self.delt_vert}
        self.send_command(command)

    def move_down(self):
        command = {'joint': 'joint_lift', 'delta': -self.delt_vert}
        self.send_command(command)

    def extend(self):
        command = {'joint': ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'], 'delta': -self.delt_horiz}
        self.send_arm_command(command)

    def retract(self):
        command = {'joint': ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'], 'delta': self.delt_horiz}
        self.send_arm_command(command)

    def send_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.arm_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)
        
    def send_arm_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            trajectory_goal.trajectory.joint_names = command['joint']
            point.positions = []
            for j_name in trajectory_goal.trajectory.joint_names:
                joint_index = joint_state.name.index(j_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta/len(trajectory_goal.trajectory.joint_names)
                point.positions.append(new_value)
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.arm_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")

class GripperController():
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None, dex_wrist=False):
        if not dex_wrist:
            self.areas = [
                {
                    "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                    "callback" : self.open_gripper
                },
                {
                    "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                    "callback" : self.close_gripper
                },
                {
                    "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                    "callback" : self.turn_left
                },
                {
                    "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                    "callback" : self.turn_right
                }
            ]
        self.grip_factor = 0.05
        self.yaw_factor = 0.1
        self.parent = parent
        self.joint_states = None
        self.arm_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        server_reached = self.arm_client.wait_for_server(timeout=rospy.Duration(60.0))
        self.joints_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        #self.head_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)

    def joint_states_cb(self, data):
        self.joint_states = data

    def mouseover_event(self, event):
        pass

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def open_gripper(self):
        command = {'joint': 'joint_gripper_finger_left', 'delta': self.grip_factor}
        self.send_command(command)

    def close_gripper(self):
        command = {'joint': 'joint_gripper_finger_left', 'delta': -self.grip_factor}
        self.send_command(command)

    def turn_left(self):
        command = {'joint': 'joint_wrist_yaw', 'delta': self.yaw_factor}
        self.send_command(command)

    def turn_right(self):
        command = {'joint': 'joint_wrist_yaw', 'delta': -self.yaw_factor}
        self.send_command(command)

    def send_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.arm_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)
        

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")


# class KeyboardPublisher:
#     def __init__(self):
#         rospy.init_node('keyboard_publisher', anonymous=True)

#         # Create a publisher for the keyboard input
#         self.keyboard_pub = rospy.Publisher('/keyboard_input', String, queue_size=10)

#         # Set up the keyboard listener
#         self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
#         self.keyboard_listener.start()

#         # Spin to keep the node alive
#         rospy.spin()

#     def on_key_press(self, key):
#         try:
#             # Get the character representation of the key
#             key_char = key.char

#             # Publish the pressed key to the topic
#             self.keyboard_pub.publish(key_char)
#         except AttributeError:
#             # Handle special keys if needed
#             pass


class SquareButton(QPushButton):
    def __init__(self, parent=None, dimensions=(80, 80)):
        super(QPushButton, self).__init__(parent)
        self.setFixedSize(dimensions[0], dimensions[1])

class Button(QPushButton):
    ''' Creates buttons (this section you select the size and shape)'''
    def __init__(self, parent=None, dimensions=(730, 94)):
        super(QPushButton, self).__init__(parent)
        self.setFixedSize(dimensions[0], dimensions[1])


# class emotionButtonSet(QWidget):
#     def __init__(self, parent=None):
#         super(emotionButtonSet, self).__init__(parent)
#         self.pointSize = 60     
#         self.controller = KeyboardPublisher(parent=self)                                
#         self.fontD = self.font()
#         self.fontD.setPointSize(self.pointSize)

#         self.up_button = Button(parent=self)
#         # self.up_button.setFont(self.fontD)
#         self.up_button.setText("Happy")
        
#         self.down_button = Button(parent=self)
#         # self.down_button.setFont(self.fontD)
#         self.down_button.setText("Sad")

#         self.right_button = Button(parent=self)
        
#         # self.right_button.setFont(self.fontD)
#         self.right_button.setText("angry")

#         self.left_button = Button(parent=self)
#         # self.left_button.setFont(self.fontD)
#         self.left_button.setText("Confuse")

#         self.test1 = "w"


#         self.middle_button = Button(parent=self)
#         # self.middle_button.setFont(self.fontD)
#         self.middle_button.setText("Base")

        # if self.controller == 'h':
        #     self.up_button.clicked.connect(self.controller)
        # if self.controller == 'b':
        #     self.down_button.clicked.connect(self.controller)
        # if self.controller == 'n':
        #     self.right_button.clicked.connect(self.controller)
        # if self.controller == 'c':
        #     self.left_button.clicked.connect(self.controller)    
        # if self.controller == 'o':
        #     self.middle_button.clicked.connect(self.controller)        


        # self.main_layout = QVBoxLayout()
        # self.middle_widget = QWidget()
        # self.middle_layout = QHBoxLayout()

        # self.middle_layout.addWidget(self.left_button)
        # self.middle_layout.addWidget(self.right_button)
        # self.main_layout.addWidget(self.middle_widget)      
        # self.main_layout.addWidget(self.up_button) 
        # self.main_layout.addWidget(self.down_button)
        # self.main_layout.addWidget(self.middle_widget)    
        # self.middle_layout.setAlignment(self.left_button, Qt.AlignLeft)
        # self.middle_layout.setAlignment(self.right_button, Qt.AlignRight)
        # self.middle_widget.setLayout(self.middle_layout)
        # self.main_layout.setAlignment(self.up_button, Qt.AlignTop)
        # self.main_layout.setAlignment(self.down_button, Qt.AlignBottom)

        # self.setLayout(self.main_layout)
        # self.setFixedSize(300, 300)


class eyedirection(QWidget):
    def __init__(self):
        self.values = rospy.Subscriber('/head_camera_jointstate', Float64MultiArray, self.dataa)
        self.a = None
        self.b = None
    def dataa(self, msg):
        MoFlL = 0.336
        MoLB = 0.026 
        MoBR = -0.252
        MoRFr = -0.563
        MoFr = -0.908


        vFdD = -0.337
        vDB = 0.001
        vBU = 0.139
        vUFu = 0.277
        upper_limit = 0.415
        lower_limit = -1.917
        MaFlL = -1.225

        MaLB = -1.542 
        MaBR = -1.859

        MaRFr = -2.196

        MaFr = -4.045
        information = msg
        transition_counter = -.130
        horiz = information.data[0]
        vert = information.data[1] 

        if horiz < -0.908:
            manip = True

        elif horiz >= -0.908:
            manip = False

        if manip:
            if vert >= vDB and vert < vBU and horiz <= MaLB and horiz > MaBR:
                # print('eyes center')
                self.a = str("eyes center")
            elif vert >= vBU and vert < vUFu and horiz <= MaLB and horiz > MaBR:
                # print("slight up")
                self.a = str("slight up")
            elif vert >= vUFu and vert <= upper_limit and horiz <= MaFlL and horiz > MaRFr:
                # print("eyes up")
                self.a = str("eyes up")
            elif vert >= vFdD and vert < vDB and horiz <= MaLB and horiz > MaBR:
                # print("slight down")
                self.a = str("slight down")
            elif vert >= lower_limit and vert < vFdD and horiz <= MaFlL and horiz > MaRFr:
                # print("eyes down")
                self.a = str("eyes down")
            elif vert >= vDB and vert < vBU and horiz <= MaFlL and horiz > MaLB:
                # print("slight right")
                self.a = str("slight right")
            elif vert >= vFdD and vert <= vUFu and horiz >= MaFlL and horiz < MoFr:
                # print("eyes right")
                self.a = str("eyes right")
            elif vert >= vDB and vert < vBU and horiz <= MaBR and horiz > MaRFr:
                # print("slight left")
                self.a = str("slight left")
            elif vert >= vFdD and vert <= vUFu and horiz <= MaRFr and horiz > MaFr:
                # print("eyes left")
                self.a = str("eyes left")
            elif vert >= vFdD and vert < vDB and horiz <= MaBR and horiz > MaRFr or vert >= lower_limit and vert < vFdD and horiz <= MaRFr and horiz > MaFr:
                # print("eyes down&left")
                self.a = str("eyes down&left")
            elif (vDB > vert >= vFdD) or (vFdD > vert >= lower_limit and MaFlL < horiz <= MoFr):
                # print("eyes down&right ")
                self.a = str("eyes down&right")
            return self.a
        if not manip:
            if vert >= vDB and vert < vBU and horiz <= MoLB and horiz > MoBR:
                # print("Front: eyes center")
                self.a = str("Front: eyes center")
            elif vert >= vBU and vert < vUFu and horiz <= MoLB and horiz > MoBR:
                # print("Front: Slight up")
                self.a = str("Front: Slight up")
            elif vert >= vUFu and vert <= upper_limit and horiz <= MoFlL and horiz > MoRFr:
                # print("Front: Eyes up")
                self.a = str("Front: Eyes up")
            elif vert >= vFdD and vert < vDB and horiz <= MoLB and horiz > MoBR:
                # print("Front: Slight down")
                self.a = str("Front: Slight down")
            elif vert >= lower_limit and vert < vFdD and horiz <= MoFlL and horiz > MoRFr:
                # print("Front: Eyes down")
                self.a = str("Front: Eyes down")
            elif vert >= vDB and vert < vBU and horiz <= MoFlL and horiz > MoLB:
                # print("Front: Slight Right")
                self.a = str("Front: Slight Right")
            elif vert >= vFdD and vert <= vUFu and horiz >= MoFlL and horiz <= 1.67:
                # print("Front: Eyes right")
                self.a = str("Front: Eyes right")
            elif vert >= vDB and vert < vBU and horiz <= MoBR and horiz > MoRFr:
                # print("Front: Slight left")
                self.a = str("Front: Slight left")
            elif vert >= vFdD and vert <= vUFu and horiz <= MoRFr and horiz > MoFr:
                # print("Front: Eyes Left")
                self.a = str("Front: Eyes Left")
            elif (vert >= vBU and vert < vUFu and horiz <= MoBR and horiz > MoRFr) or (vert >= vUFu and vert < upper_limit and horiz <= MoRFr and horiz > MoFr ):
                # print("Front: Eyes up&left")
                self.a = str("Front: Eyes up&left")
            elif  (vert >= vBU and vert < vUFu and horiz <= MoFlL and horiz > MoLB) or (vert >= vUFu and vert < upper_limit and horiz >= MoFlL and horiz <= 1.67):
                # print("Front: Eyes up&right")
                self.a = str("Front: Eyes up&right")
            elif (vert >= vFdD and vert < vDB and horiz <= MoBR and horiz > MoRFr) or (vert >= lower_limit and vert < vFdD and horiz <= MoRFr and horiz > MoFr):
                # print("Front: Down&Left")
                self.a = str("Front: Down&Left")
            elif (vert >= vFdD and vert < vDB and horiz <= MoFlL and horiz > MoLB) or (vert >= lower_limit and vert < vFdD and horiz >= MoFlL and horiz <= 1.67):
                # print("Front:Down&Right")
                self.a = str("Front:Down&Right")
            return self.a
        
class CameraButtonSet(QWidget):
    def __init__(self, parent=None):
        super(CameraButtonSet, self).__init__(parent)
        self.controller = CameraController(parent=self)
        self.up_button = SquareButton(parent=self)
        self.up_button.setText("Tilt Up")
        self.down_button = SquareButton(parent=self)
        self.down_button.setText("Tilt Down")
        self.left_button = SquareButton(parent=self)
        self.left_button.setText("Pan Left")
        self.right_button = SquareButton(parent=self)
        self.right_button.setText("Pan Right")

        self.up_button.clicked.connect(self.controller.tilt_up)
        self.down_button.clicked.connect(self.controller.tilt_down)
        self.left_button.clicked.connect(self.controller.turn_left)
        self.right_button.clicked.connect(self.controller.turn_right)

        self.main_layout = QVBoxLayout()
        self.middle_widget = QWidget()
        self.middle_layout = QHBoxLayout()

        self.middle_layout.addWidget(self.left_button)
        self.middle_layout.addWidget(self.right_button)
        self.middle_layout.setAlignment(self.left_button, Qt.AlignLeft)
        self.middle_layout.setAlignment(self.right_button, Qt.AlignRight)
        self.middle_widget.setLayout(self.middle_layout)

        self.main_layout.addWidget(self.up_button)
        self.main_layout.addWidget(self.middle_widget)
        self.main_layout.addWidget(self.down_button)

        self.main_layout.setAlignment(self.up_button, Qt.AlignTop)
        self.main_layout.setAlignment(self.down_button, Qt.AlignBottom)

        self.main_layout.setAlignment(self.up_button, Qt.AlignCenter)
        self.main_layout.setAlignment(self.down_button, Qt.AlignCenter)

        self.setLayout(self.main_layout)
        self.setFixedSize(300, 300)


#Based on the top answer to this question https://stackoverflow.com/questions/57204782/show-an-opencv-image-with-pyqt5
class DisplayImageWidget(QWidget):
    def __init__(self, parent=None):
        super(DisplayImageWidget, self).__init__(parent)


        #self.button = QPushButton('Show picture')
        #self.button.clicked.connect(self.show_image)
        self.image_frame = QLabel()
        self.image_frame.setFixedSize(432, 768)

        #Needed to handle mouse move events
        self.setMouseTracking(True)

        self.nav_controller = NavigationController(parent=self)
        self.arm_controller = ArmController(parent=self)
        self.grip_controller = GripperController(parent=self)

        self.mode = "camera"

        self.available_modes = {
            "camera" : {
                "show_function" : self.only_show_image,
                "controller" : None
            }
        }

        self.image_frame.mousePressEvent = self.click_event
        self.setAttribute(QtCore.Qt.WA_Hover)
        #self.image_frame.mouseMoveEvent = self.mouseover_event


        self.layout = QVBoxLayout()
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

    def set_mode(self, mode):
        if mode in self.available_modes.keys():
            self.mode = mode
            logging.debug(f"Switching to mode {mode}")
        else:
            self.mode = "camera"
            logging.warning(f"Interface has no mode \"{mode}\"")

    def mouseMoveEvent(self, event):
        if self.available_modes[self.mode]["controller"] != None:
            self.available_modes[self.mode]["controller"].mouseover_event(event)

    def click_event(self, event):
        if self.available_modes[self.mode]["controller"] != None:
            self.available_modes[self.mode]["controller"].click_event(event)

    def only_show_image(self, cv_image):
        cv_image = cv2.resize(cv_image, (self.image_frame.width(), self.image_frame.height()), interpolation=cv2.INTER_LINEAR)
        cv_image = self.draw_shapes(cv_image)
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))

    def show_navigation(self, cv_image):
        cv_image = cv2.resize(cv_image, (self.image_frame.width(), self.image_frame.height()), interpolation=cv2.INTER_LINEAR)
        cv_image = self.draw_shapes(cv_image)
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))

    def draw_shapes(self, cv_image):
        overlay = cv_image.copy()
        alpha = 0.5
        dimensions = self.image_frame.size()
        w = dimensions.width()
        h = dimensions.height()
        for area in self.available_modes[self.mode]["controller"].areas:
            points = area["geometry"].exterior.coords
            for i in range(len(points)-1):
                p1 = points[i+1]
                p2 = points[i]
                cv2.line(overlay, (int(p1[0]*w), int(p1[1]*h)), (int(p2[0]*w), int(p2[1]*h)), (255, 255, 255), 4)
        cv_image = cv2.addWeighted(overlay, alpha, cv_image, 1-alpha, 0)
        return cv_image

    def show_image_by_mode(self, cv_image):
        self.available_modes[self.mode]["show_function"](cv_image)

    @QtCore.pyqtSlot()
    def show_image(self, image):
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(image))


class HomePage(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()
        self.layout = QVBoxLayout()
        self.top_bar = QLabel()
        self.top_bar.setText("Stretch Touch Based Controller")
        
        self.navmode_button = QPushButton(text="Navigation Mode")
        self.manipulation_button = QPushButton(text="Manipulation Mode")

        self.navmode_button.setFixedHeight(80)
        self.manipulation_button.setFixedHeight(80)
        
        self.button_widget = QWidget()
        self.button_layout = QHBoxLayout()
        self.button_layout.addWidget(self.navmode_button)
        self.button_layout.addWidget(self.manipulation_button)
        self.button_widget.setLayout(self.button_layout)

        self.layout.addWidget(self.top_bar)
        self.layout.addWidget(self.button_widget)

        self.setLayout(self.layout)

class NavigationPage(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.vid_bridge = CvBridge()

        self.vid_subscriber = rospy.Subscriber("camera/color/image_raw", Image, self.camera_cb)

        self.vid_label = QLabel(text= "Navigation Mode")
        self.vid_label.setMaximumHeight(20)
        self.vid_widget = DisplayImageWidget(parent=self)

        self.vid_widget.available_modes["navigation"] = {"show_function" : self.vid_widget.show_navigation, "controller" : NavigationController(parent=self.vid_widget)}

        self.vid_widget.set_mode("navigation")

        self.manipulation_button = QPushButton(text="Manipulation Mode")
        self.manipulation_button.setFixedHeight(80)
        self.manipulation_button.setMinimumWidth(500)

        self.camera_layout = QVBoxLayout()

        self.main_layout = QHBoxLayout()
        self.camera_widget = QWidget()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_labell)
        self.timer.start(500)

        self.camera_layout.addWidget(self.vid_label)
        self.camera_layout.addWidget(self.vid_widget)
        self.camera_layout.addWidget(self.manipulation_button)


        self.valuee = 0
        self.labell = QLabel("Initial Text")
        self.labell.move(400,0)
        self.camera_layout.addStretch()
    
        self.labelll = QLabel("H = Happy \nO = Normal \nN = Angry \nB = Sad \nC = Confuse")

        self.camera_layout.setAlignment(self.vid_label, Qt.AlignHCenter)
        self.camera_layout.setAlignment(self.vid_widget, Qt.AlignHCenter)
        self.camera_layout.setAlignment(self.manipulation_button, Qt.AlignHCenter)
        # self.camera_layout.setAlignment(self.labell, Qt.Align)

        self.cam_buttons = CameraButtonSet(parent=self)

        self.camera_widget.setLayout(self.camera_layout)


        self.main_layout.addWidget(self.camera_widget)
        self.main_layout.addWidget(self.cam_buttons)
        self.main_layout.addWidget(self.labell)
        self.main_layout.addWidget(self.labelll)

        self.direction = eyedirection()
        self.main_layout.setAlignment(self.cam_buttons, Qt.AlignHCenter)
        self.main_layout.setAlignment(self.labell, Qt.AlignHCenter)
        # self.main_layout.setAlignment(self.labelll, Qt.Align)
        self.setLayout(self.main_layout)


    def update_labell(self):
    # Read content from the text file
        self.valuee = self.direction.a
        # print(self.valuee)
        self.labell.setText(str(self.valuee))


    def camera_cb(self, data):
        cv_image = cv2.rotate(self.vid_bridge.imgmsg_to_cv2(data), cv2.ROTATE_90_CLOCKWISE)
        self.vid_widget.show_image_by_mode(cv_image)


class ManipulationPage(QWidget):
    def __init__(self):
        super(QWidget, self).__init__()

        self.layout = QVBoxLayout()
        #self.top_label = QLabel(text="Manipulator Mode")
        #self.top_label.setMaximumHeight(20)
        
        self.vid_bridge = CvBridge()
        
        self.videos_widget = QWidget()
        self.videos_layout = QHBoxLayout()

        self.main_camera_widget = QWidget()
        self.main_camera_label = QLabel(text="Arm Control")
        self.main_camera_label.setFixedHeight(20)

        self.main_camera = DisplayImageWidget(parent=self)
        self.main_camera.available_modes["arm"] = {"show_function" : self.main_camera.show_navigation, "controller" : ArmController(parent=self.main_camera)}
        self.main_camera.set_mode("arm")

        self.main_camera_layout = QVBoxLayout()
        self.main_camera_layout.addWidget(self.main_camera_label)
        self.main_camera_layout.addWidget(self.main_camera)
        #self.main_camera_layout.addStretch()

        self.main_camera_layout.setAlignment(self.main_camera_label, Qt.AlignHCenter)

        self.arm_camera_widget = QWidget()
        self.arm_camera_label = QLabel(text="Gripper Control")
        self.arm_camera_label.setFixedHeight(20)

        self.arm_camera = DisplayImageWidget(parent=self)
        self.arm_camera.image_frame.setFixedSize(960, 540)
        self.arm_camera.available_modes["gripper"] = {"show_function" : self.arm_camera.show_navigation, "controller" : GripperController(parent=self.arm_camera)}
        self.arm_camera.set_mode("gripper")


        self.arm_camera_layout = QVBoxLayout()
        self.arm_camera_layout.addWidget(self.arm_camera_label)
        self.arm_camera_layout.addWidget(self.arm_camera)

        self.cam_buttons = CameraButtonSet(parent=self)
        # self.emotions = emotionButtonSet(parent=self)

        self.arm_camera_layout.addWidget(self.cam_buttons)
        # self.arm_camera_layout.addWidget(self.emotions)
        self.arm_camera_layout.setAlignment(self.arm_camera_label, Qt.AlignHCenter)

        self.main_camera_widget.setLayout(self.main_camera_layout)
        self.arm_camera_widget.setLayout(self.arm_camera_layout)
##########################################################################################################################
        self.arm_camera_layout.setAlignment(self.cam_buttons, Qt.AlignCenter)

        self.value = 0
        self.label = QLabel("Initial Text")

        self.arm_camera_layout.addWidget(self.label)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_label)
        self.timer.start(500)


        self.videos_layout.addWidget(self.arm_camera_widget)
        self.videos_layout.addWidget(self.main_camera_widget)
        self.videos_widget.setLayout(self.videos_layout)

        self.navigation_button = QPushButton(text="Navigation Mode")
        self.navigation_button.setFixedHeight(80)

        #self.layout.addWidget(self.top_label)
        self.layout.addWidget(self.videos_widget)
        self.layout.addWidget(self.navigation_button)
        self.layout.addStretch()
        self.direction = eyedirection()
        self.setLayout(self.layout)

        self.vid_subscriber = rospy.Subscriber("camera/color/image_raw", Image, self.main_camera_cb)

        self.arm_cam_thread = threading.Thread(target=self.run_arm_camera, daemon=True)
        self.arm_cam_thread.start()

    def update_label(self):
    # Read content from the text file
        self.value = self.direction.a
        self.label.setText(str(self.value))

    def main_camera_cb(self, data):
        cv_image = cv2.rotate(self.vid_bridge.imgmsg_to_cv2(data), cv2.ROTATE_90_CLOCKWISE)
        self.main_camera.show_image_by_mode(cv_image)

    def run_arm_camera(self):
        vid = cv2.VideoCapture(6)
        #vid = cv2.VideoCapture(0)
        while(True):
            # Capture the video frame
            # by frame
            _, frame = vid.read()
            self.arm_camera.show_image_by_mode(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))



#Main window class
class MainWindow(QMainWindow):
    def __init__(self):
        super(QWidget, self).__init__()
        
        self.vid_bridge = CvBridge()

        self.vid_widget = DisplayImageWidget(parent=self)

        self.main_widget = QStackedWidget()

        self.home_page = HomePage()
        self.nav_page = NavigationPage()
        self.manipulation_page = ManipulationPage()

        self.main_widget.addWidget(self.home_page)
        self.main_widget.addWidget(self.nav_page)
        self.main_widget.addWidget(self.manipulation_page)

        self.home_page.navmode_button.clicked.connect(lambda: self.change_page(1))
        self.home_page.manipulation_button.clicked.connect(lambda: self.change_page(2))
        

        self.manipulation_page.navigation_button.clicked.connect(lambda: self.change_page(1))
        self.nav_page.manipulation_button.clicked.connect(lambda: self.change_page(2))
        


        self.setCentralWidget(self.main_widget)

        
    def change_page(self, i):
        self.main_widget.setCurrentIndex(i)        


    def camera_cb(self, data):
        cv_image = cv2.rotate(self.vid_bridge.imgmsg_to_cv2(data), cv2.ROTATE_90_CLOCKWISE)
        self.vid_widget.show_image_by_mode(cv_image)


    def change_video_mode(self, mode):
        self.vid_widget.set_mode(mode)

if __name__=="__main__":
    #logging.basicConfig(format='%(levelname)s %(asctime)s: %(message)s',filename=f"{os.path.dirname(os.path.abspath(__file__))}/logs/qt_interface_{time.strftime('%y_%m_%d:%H_%M_%S', time.localtime(time.time()))}.log", level=logging.INFO, datefmt="%y-%m-%d:%h-%m-%s")
    #logging.info("Starting application")

    # try:
    #     KeyboardPublisher()
    # except rospy.ROSInterruptException:
    #     pass

    rospy.init_node("qt_interface", anonymous=True)

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    #logging.info("Application started")

    app.exec()
