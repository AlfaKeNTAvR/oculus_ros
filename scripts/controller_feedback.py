#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import transformations

from geometry_msgs.msg import (Pose)

from oculus_ros.msg import (
    ControllerInput,
    ControllerButtons,
    ControllerJoystick,
)


class ControllerFeedback:
    """
    
    """

    def __init__(
        self,
        controller_side,
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

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__pose = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/oculus/pose',
            Pose,
            queue_size=1,
        )
        self.__buttons = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/oculus/buttons',
            ControllerButtons,
            queue_size=1,
        )
        self.__joystick = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/oculus/joystick',
            ControllerJoystick,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}ControllerInfo',
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
        pose_message.position.x = self.__controller_pose.position.z
        pose_message.position.y = -1 * self.__controller_pose.position.x
        pose_message.position.z = self.__controller_pose.position.y

        # Converts quaternions from left-handed coordinate system to
        # right-handed coordinate system:
        # 1. W stays the same.
        # 2. New X is negative old Z.
        # 3. New Y is old X.
        # 4. New Z is negative old Y.
        orientation_quaternion = np.array(
            [
                self.__controller_pose.orientation.w,
                -self.__controller_pose.orientation.z,
                self.__controller_pose.orientation.x,
                -self.__controller_pose.orientation.y,
            ]
        )

        pose_message.orientation.w = orientation_quaternion[0]
        pose_message.orientation.x = orientation_quaternion[1]
        pose_message.orientation.y = orientation_quaternion[2]
        pose_message.orientation.z = orientation_quaternion[3]

        self.__pose.publish(pose_message)

    def node_shutdown(self):
        """
        
        """

        print(
            f'\n/{self.CONTROLLER_SIDE}/controller_feedback: node is shutting down...\n'
        )

        print(
            f'\n/{self.CONTROLLER_SIDE}/controller_feedback: node is shut down.\n'
        )


def main():
    """

    """

    rospy.init_node('controller_feedback')

    controller_side = rospy.get_param(
        param_name=f'{rospy.get_name()}/controller_side',
        default='right',
    )

    controller = ControllerFeedback(controller_side=controller_side)

    rospy.on_shutdown(controller.node_shutdown)

    print(f'\n/{controller_side}/controller_feedback: ready.\n')

    while not rospy.is_shutdown():
        controller.main_loop()


if __name__ == '__main__':
    main()