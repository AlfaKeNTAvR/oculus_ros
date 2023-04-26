#!/usr/bin/env python
"""

"""

import rospy

from geometry_msgs.msg import (Pose)
from std_msgs.msg import (Bool)

from oculus.msg import (ControllerButtons)


class OculusKinovaMapping:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        controller_side='right',
        tracking_mode='press',
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        if tracking_mode not in ['hold', 'press']:
            raise ValueError(
                'tracking_mode should be either "hold" or "press".'
            )

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.CONTROLLER_SIDE = controller_side
        self.TRACKING_MODE = tracking_mode

        # # Private variables:
        self.__oculus_pose = Pose()
        self.__oculus_buttons = ControllerButtons()
        self.__tracking_state_machine_state = 0

        # # Public variables:
        self.tracking = False

        # # ROS node:

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__kinova_tracking = rospy.Publisher(
            f'{self.ROBOT_NAME}/controller/tracking',
            Bool,
            queue_size=1,
        )
        self.__kinova_pose = rospy.Publisher(
            f'{self.ROBOT_NAME}/controller/pose',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/pose',
            Pose,
            self.__oculus_pose_callback,
        )
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_pose_callback(self, message):
        """

        """

        self.__oculus_pose = message

        # TODO: Proper orientation support.
        self.__oculus_pose.orientation.w = 0.6532815
        self.__oculus_pose.orientation.x = -0.2705981
        self.__oculus_pose.orientation.y = -0.2705981
        self.__oculus_pose.orientation.z = 0.6532815

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

        if self.TRACKING_MODE == 'hold':
            self.tracking = self.__oculus_buttons.grip_button

    # # Private methods:
    def __tracking_state_machine(self):
        """
        
        """

        # State 0: Trigger button was pressed.
        if (
            self.__oculus_buttons.grip_button
            and self.__tracking_state_machine_state == 0
        ):
            self.__tracking_state_machine_state = 1

        # State 1: Trigger button was released. Tracking is activated.
        elif (
            not self.__oculus_buttons.grip_button
            and self.__tracking_state_machine_state == 1
        ):
            self.tracking = True
            self.__tracking_state_machine_state = 2

        # State 2: Trigger button was pressed. Tracking is deactivated.
        elif (
            self.__oculus_buttons.grip_button
            and self.__tracking_state_machine_state == 2
        ):
            self.tracking = False
            self.__tracking_state_machine_state = 3

        # State 3: Trigger button was released.
        elif (
            not self.__oculus_buttons.grip_button
            and self.__tracking_state_machine_state == 3
        ):
            self.__tracking_state_machine_state = 0

    # # Public methods:
    def main_loop(self):
        """
        
        """

        if self.TRACKING_MODE == 'press':
            self.__tracking_state_machine()
        self.__kinova_tracking.publish(self.tracking)

        self.__kinova_pose.publish(self.__oculus_pose)


def node_shutdown():
    """
    
    """

    print('\nNode is shutting down...\n')

    # TODO: Stop arm motion.

    print('\nNode is shut down.\n')


def main():
    """

    """

    # # ROS node:
    rospy.init_node('oculus_kinova_mapping')
    rospy.on_shutdown(node_shutdown)

    right_arm_mapping = OculusKinovaMapping(
        robot_name='my_gen3',
        controller_side='right',
        tracking_mode='press',
    )

    print('\nOculus-Kinova mapping is ready.\n')

    while not rospy.is_shutdown():
        right_arm_mapping.main_loop()


if __name__ == '__main__':
    main()
