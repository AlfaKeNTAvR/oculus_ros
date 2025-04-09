#!/usr/bin/env python
"""Implements Oculus Quest 2 headset feedback module.

TODO: Add detailed description.

Author (s): 
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2024.
    
"""

import rospy
import numpy as np
import math
from ast import (literal_eval)
import tf.transformations as tf

from std_msgs.msg import (
    Bool,
    Float64,
)
from geometry_msgs.msg import (Pose)

from oculus_ros.msg import (ControllerInput)


class ControllerFeedback:
    """
    
    """

    def __init__(self,):
        """
        
        """

        # # Private constants:

        # # Public constants:

        # # Private variables:
        self.__headset_pose = Pose()

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/headset_feedback/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'unity_ros': False,
        }

        self.__dependency_status_topics = {
            'unity_ros':
                rospy.Subscriber(
                    f'/headsetInfo',
                    ControllerInput,
                    self.__headset_callback,
                ),
        }

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__pose = rospy.Publisher(
            f'/headset_feedback/pose',
            Pose,
            queue_size=1,
        )
        self.__pitch = rospy.Publisher(
            '/pitch_motor_pid/setpoint',
            Float64,
            queue_size=1,
        )
        self.__yaw = rospy.Publisher(
            '/yaw_motor_pid/setpoint',
            Float64,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/headsetInfo',
            ControllerInput,
            self.__headset_callback,
        )

    # # Dependency status callbacks:

    # # Service handlers:

    # # Topic callbacks:
    def __headset_callback(self, message):
        """

        """

        self.__headset_pose.position.x = message.controller_pos_x
        self.__headset_pose.position.y = message.controller_pos_y
        self.__headset_pose.position.z = message.controller_pos_z

        self.__headset_pose.orientation.w = message.controller_rot_w
        self.__headset_pose.orientation.x = message.controller_rot_x
        self.__headset_pose.orientation.y = message.controller_rot_y
        self.__headset_pose.orientation.z = message.controller_rot_z

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
                        (f'/headset_feedback: '
                         f'lost connection to {key}!')
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
                    f'/headset_feedback:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m/headset_feedback: ready.\033[0m',)

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

    def publish_pose(self):
        """

        """

        pose_message = Pose()

        # Transforms the headset input position from Left-Handed Coordinate
        # system to Right-handed (Global) Coordinate system:
        # 1. Swap y and z axis.
        # 2. Swap x and new y (which was z) to have x facing forward.
        # 3. Negate new y (which is x) to make it align with global coordinate
        # system.
        pose_message.position.x = self.__headset_pose.position.z
        pose_message.position.y = -1 * self.__headset_pose.position.x
        pose_message.position.z = self.__headset_pose.position.y

        # Converts quaternions from left-handed coordinate system to
        # right-handed coordinate system:
        # 1. W stays the same.
        # 2. New X is negative old Z.
        # 3. New Y is old X.
        # 4. New Z is negative old Y.
        orientation_quaternion = np.array(
            [
                self.__headset_pose.orientation.w,
                -self.__headset_pose.orientation.z,
                self.__headset_pose.orientation.x,
                -self.__headset_pose.orientation.y,
            ]
        )

        pose_message.orientation.w = orientation_quaternion[0]
        pose_message.orientation.x = orientation_quaternion[1]
        pose_message.orientation.y = orientation_quaternion[2]
        pose_message.orientation.z = orientation_quaternion[3]

        rpy_radians = tf.euler_from_quaternion(
            [
                orientation_quaternion[1],
                orientation_quaternion[2],
                orientation_quaternion[3],
                orientation_quaternion[0],
            ]
        )
        rpy_deg = np.rad2deg(np.array(rpy_radians))

        float64_message = Float64()
        float64_message.data = rpy_deg[1] + 30
        self.__pitch.publish(float64_message)

        float64_message = Float64()
        float64_message.data = rpy_deg[2]
        self.__yaw.publish(float64_message)

        self.__pose.publish(pose_message)

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'/headset_feedback: node is shutting down...',)

        rospy.loginfo_once(f'/headset_feedback: node has shut down.',)


def main():
    """

    """

    rospy.init_node(
        'headset_feedback',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    headset = ControllerFeedback()

    rospy.on_shutdown(headset.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        headset.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
