#!/usr/bin/env python
"""

"""

import rospy
import numpy as np

from geometry_msgs.msg import (Pose)
from std_msgs.msg import (
    Bool,
    Float32MultiArray,
)

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
        self.__oculus_position = np.array([0.0, 0.0, 0.0])
        self.__oculus_orientation = np.array([1.0, 0.0, 0.0, 0.0])
        self.__oculus_buttons = ControllerButtons()
        self.__tracking_state_machine_state = 0

        # # Public variables:
        self.tracking = False

        # # ROS node:
        rospy.init_node(f'oculus_{controller_side}_to_kinova_{robot_name}')
        rospy.on_shutdown(self.__node_shutdown)

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
            f'oculus/{self.CONTROLLER_SIDE}/position_gcs',
            Float32MultiArray,
            self.__oculus_position_callback,
        )
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_position_callback(self, message):
        """

        """

        self.__oculus_position = np.array(message.data)

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

        if self.TRACKING_MODE == 'hold':
            self.tracking = self.__oculus_buttons.grip_button

    # # Private methods:
    def __node_shutdown(self):
        """
        
        """

        pass

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

        self.publish_pose()

    def publish_pose(self):
        """
        
        """

        # Form the arm pose message.
        pose_message = Pose()
        pose_message.position.x = self.__oculus_position[0]
        pose_message.position.y = self.__oculus_position[1]
        pose_message.position.z = self.__oculus_position[2]

        pose_message.orientation.w = self.__oculus_orientation[0]
        pose_message.orientation.x = self.__oculus_orientation[1]
        pose_message.orientation.y = self.__oculus_orientation[2]
        pose_message.orientation.z = self.__oculus_orientation[3]

        self.__kinova_pose.publish(pose_message)


def main():
    """
    
    """

    right_arm_mapping = OculusKinovaMapping(
        robot_name='my_gen3',
        controller_side='right',
        tracking_mode='press',
    )

    while not rospy.is_shutdown():
        right_arm_mapping.main_loop()


if __name__ == '__main__':
    main()
