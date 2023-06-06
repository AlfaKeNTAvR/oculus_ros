#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import transformations

from std_msgs.msg import (Bool)
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

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/is_initialized',
            Bool,
            queue_size=1,
        )

        # TODO: Add unity_ros as a dependency.
        self.__dependency_status = {
            'unity_ros': False,
        }

        self.__dependency_status_topics = {
            'unity_ros':
                rospy.Subscriber(
                    f'/{self.CONTROLLER_SIDE}ControllerInfo',
                    ControllerInput,
                    self.__controller_callback,
                ),
        }

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__pose = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/pose',
            Pose,
            queue_size=1,
        )
        self.__buttons = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/buttons',
            ControllerButtons,
            queue_size=1,
        )
        self.__joystick = rospy.Publisher(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/joystick',
            ControllerJoystick,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}ControllerInfo',
            ControllerInput,
            self.__controller_callback,
        )

    # # Dependency status callbacks:

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

        self.__dependency_status['unity_ros'] = True

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (
                            f'/{self.CONTROLLER_SIDE}/controller_feedback: '
                            f'lost connection to {key}!'
                        )
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key}...'

            rospy.logwarn_throttle(
                15,
                (
                    f'/{self.CONTROLLER_SIDE}/controller_feedback:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.CONTROLLER_SIDE}/controller_feedback: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

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

        rospy.loginfo_once(
            f'/{self.CONTROLLER_SIDE}/controller_feedback: node is shutting down...',
        )

        rospy.loginfo_once(
            f'/{self.CONTROLLER_SIDE}/controller_feedback: node has shut down.',
        )


def main():
    """

    """

    rospy.init_node(
        'controller_feedback',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    controller_side = rospy.get_param(
        param_name=f'{rospy.get_name()}/controller_side',
        default='right',
    )

    controller = ControllerFeedback(controller_side=controller_side)

    rospy.on_shutdown(controller.node_shutdown)

    while not rospy.is_shutdown():
        controller.main_loop()


if __name__ == '__main__':
    main()