#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from pynput import keyboard

from playsound import playsound

class KeyboardPublisher:
    def __init__(self):
        rospy.init_node('keyboard_publisher', anonymous=True)

        # Create a publisher for the keyboard input
        self.keyboard_pub = rospy.Publisher('keyboard_input', String, queue_size=10)

        # Set up the keyboard listener
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press)
        self.keyboard_listener.start()

        # Spin to keep the node alive
        rospy.spin()

    def on_key_press(self, key):
        try:
            # Get the character representation of the key
            key_char = key.char

            # Publish the pressed key to the topic
            self.keyboard_pub.publish(key_char)

            if key.char == 't':
                playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/robot_sound.mp3')

        except AttributeError:
            # Handle special keys if needed
            pass

if __name__ == '__main__':
    try:
        KeyboardPublisher()
    except rospy.ROSInterruptException:
        pass
