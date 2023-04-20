#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from positional_control import utils
from ROS_TCP_Endpoint_msgs.msg import ControllerInput
from std_msgs.msg import *
from oculus.msg import Joystick, ControllerButtons


class Oculus:

    def __init__(self):

        # Subscribers
        rospy.Subscriber(
            "rightControllerInfo",
            ControllerInput,
            self.right_callback,
        )
        rospy.Subscriber(
            "leftControllerInfo",
            ControllerInput,
            self.left_callback,
        )

        # Publishers
        self.input_position_gcs_pub = rospy.Publisher(
            'oculus/right/position_gcs',
            Float32MultiArray,
            queue_size=1,
        )
        self.orientation_gcs_pub = rospy.Publisher(
            'oculus/right/orientation_gcs',
            Float32MultiArray,
            queue_size=1,
        )
        self.right_buttons_pub = rospy.Publisher(
            'oculus/right/buttons',
            ControllerButtons,
            queue_size=1,
        )
        self.right_joystick_pub = rospy.Publisher(
            'oculus/right/joystick',
            Joystick,
            queue_size=1,
        )

        # TODO: Add left controller position and orientation.
        self.left_buttons_pub = rospy.Publisher(
            'oculus/left/buttons',
            ControllerButtons,
            queue_size=1,
        )
        self.left_joystick_pub = rospy.Publisher(
            'oculus/left/joystick',
            Joystick,
            queue_size=1,
        )

    def right_callback(self, data):
        """Callback function for right controller info. Transform the controller input position from 
        Left-Handed Coordinate system to Right-handed Coordinate system: 
        1) swap y and z axis;
        2) swap x and new y (which was z) to have x facing forward;
        3) negate new y (which is x) to make it align with global coordinate system.
        """

        # Transition from Left-handed CS (Unity) to Right-handed CS (Global) for controller position
        input_pos_gcs = np.array(
            [
                -1 * data.controller_pos_z,
                data.controller_pos_x,
                data.controller_pos_y,
            ]
        )

        # # ORIENTATION
        # # Raw quaternion input (Left-handed CS)
        # input_rot_gcs = np.array(
        #     [
        #         data.controller_rot_x,
        #         data.controller_rot_y,
        #         data.controller_rot_z,
        #         data.controller_rot_w,
        #     ]
        # )

        # # Transition from Left-handed CS (Unity) to Right-handed CS (Global).
        # input_rot_gcs = utils.left_to_right_handed(input_rot_gcs)

        # # More comfortable position (compensation)
        # Qy = T.quaternion_about_axis(
        #     math.radians(-45),
        #     (0, 1, 0),
        # )
        # input_rot_gcs = T.quaternion_multiply(
        #     input_rot_gcs,
        #     Qy,
        # )

        # Transition from Global CS to Kinova CS: rotate around y and z axis
        # self.input_rot_kcs = utils.global_to_kinova(input_rot_gcs)

        # Publish controller position
        controller_position = Float32MultiArray()
        controller_position.data = input_pos_gcs
        self.input_position_gcs_pub.publish(controller_position)

        # # Publish controller orientation
        # controller_orientation = Float32MultiArray()
        # controller_orientation.data = input_rot_gcs
        # self.orientation_gcs_pub.publish(controller_orientation)

        # Publish joystick position
        joystick_message = Joystick()
        joystick_message.button = data.joystickButton
        joystick_message.position_x = data.joystick_pos_x
        joystick_message.position_y = data.joystick_pos_y

        self.right_joystick_pub.publish(joystick_message)

        # Publish controller buttons status
        buttons_message = ControllerButtons()
        buttons_message.primary_button = data.primaryButton
        buttons_message.secondary_button = data.secondaryButton
        buttons_message.grip_button = data.gripButton
        buttons_message.trigger_button = data.triggerButton
        buttons_message.trigger_value = data.triggerValue

        self.right_buttons_pub.publish(buttons_message)

    # Left controller topic callback function
    def left_callback(self, data):
        """Callback function for left controller info.
        """

        # Publish joystick position
        joystick_message = Joystick()
        joystick_message.button = data.joystickButton
        joystick_message.position_x = data.joystick_pos_x
        joystick_message.position_y = data.joystick_pos_y

        self.left_joystick_pub.publish(joystick_message)

        # Publish controller buttons status
        buttons_message = ControllerButtons()
        buttons_message.primary_button = data.primaryButton
        buttons_message.secondary_button = data.secondaryButton
        buttons_message.grip_button = data.gripButton
        buttons_message.trigger_button = data.triggerButton
        buttons_message.trigger_value = data.triggerValue

        self.left_buttons_pub.publish(buttons_message)


if __name__ == '__main__':

    rospy.init_node("headset_feedback")
    Headset = Oculus()

    while not rospy.is_shutdown():
        pass
