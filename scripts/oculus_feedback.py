#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import transformations as T

from geometry_msgs.msg import (Pose)

from oculus.msg import (
    ControllerInput,
    ControllerButtons,
    ControllerJoystick,
)


class ControllerFeedback:
    """
    
    """

    def __init__(
        self,
        controller_side='right',
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        # # Private constants:

        # # Public constants:
        self.CONTROLLER_SIDE = controller_side

        # # Private variables:
        self.__controller_pose = Pose()
        self.__controller_buttons = ControllerButtons()
        self.__controller_joystick = ControllerJoystick()

        # # Public variables:

        # # ROS node:

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__pose = rospy.Publisher(
            f'oculus/{self.CONTROLLER_SIDE}/pose',
            Pose,
            queue_size=1,
        )
        self.__buttons = rospy.Publisher(
            f'oculus/{self.CONTROLLER_SIDE}/buttons',
            ControllerButtons,
            queue_size=1,
        )
        self.__joystick = rospy.Publisher(
            f'oculus/{self.CONTROLLER_SIDE}/joystick',
            ControllerJoystick,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.CONTROLLER_SIDE}ControllerInfo',
            ControllerInput,
            self.__controller_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __controller_callback(self, message):
        """

        """

        self.__controller_pose.position.x = message.controller_pos_x
        self.__controller_pose.position.y = message.controller_pos_y
        self.__controller_pose.position.z = message.controller_pos_z

        self.__controller_pose.orientation.w = message.controller_rot_w
        self.__controller_pose.orientation.x = message.controller_rot_x
        self.__controller_pose.orientation.y = message.controller_rot_y
        self.__controller_pose.orientation.z = message.controller_rot_z

        self.__controller_buttons.primary_button = message.primaryButton
        self.__controller_buttons.secondary_button = message.secondaryButton
        self.__controller_buttons.grip_button = message.gripButton
        self.__controller_buttons.trigger_button = message.triggerButton
        self.__controller_buttons.trigger_value = message.triggerValue

        self.__controller_joystick.button = message.joystickButton
        self.__controller_joystick.position_x = message.joystick_pos_x
        self.__controller_joystick.position_y = message.joystick_pos_y

    # # Private methods:

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.publish_pose()
        self.__buttons.publish(self.__controller_buttons)
        self.__joystick.publish(self.__controller_joystick)

    def publish_pose(self):
        """

        """

        pose_message = Pose()

        # Transforms the controller input position from Left-Handed Coordinate
        # system to Right-handed (Global) Coordinate system:
        # 1. Swap y and z axis.
        # 2. Swap x and new y (which was z) to have x facing forward.
        # 3. Negate new y (which is x) to make it align with global coordinate
        # system.
        pose_message.position.x = -1 * self.__controller_pose.position.z
        pose_message.position.y = self.__controller_pose.position.x
        pose_message.position.z = self.__controller_pose.position.y

        # Converts quaternions from left-handed coordinate system to
        # right-handed coordinate system:
        # 1. Convert to Euler angles.
        orientation_euler = T.euler_from_quaternion(
            np.array(
                [
                    self.__controller_pose.orientation.w,
                    self.__controller_pose.orientation.x,
                    self.__controller_pose.orientation.y,
                    self.__controller_pose.orientation.z,
                ]
            )
        )

        # 2. Swap and negate from (X, Y, Z) to (Z, -X, Y).
        orientation_euler_gcs = np.array(
            [
                orientation_euler[2],
                -1 * orientation_euler[0],
                orientation_euler[1],
            ]
        )

        # 3. Convert to a quaternion.
        orientation_quaternion = T.quaternion_from_euler(
            orientation_euler_gcs[0],
            orientation_euler_gcs[1],
            orientation_euler_gcs[2],
        )

        pose_message.orientation.w = orientation_quaternion[0]
        pose_message.orientation.x = orientation_quaternion[1]
        pose_message.orientation.y = orientation_quaternion[2]
        pose_message.orientation.z = orientation_quaternion[3]

        self.__pose.publish(pose_message)


def node_shutdown():
    """
    
    """

    print('\nNode is shutting down...\n')

    print('\nNode is shut down.\n')


def main():
    """

    """

    # # ROS node:
    rospy.init_node('oculus_feedback')
    rospy.on_shutdown(node_shutdown)

    right_controller = ControllerFeedback(controller_side='right')
    left_controller = ControllerFeedback(controller_side='left')

    print('\nOculus feedback is ready.\n')

    while not rospy.is_shutdown():
        right_controller.main_loop()
        left_controller.main_loop()


if __name__ == '__main__':
    main()