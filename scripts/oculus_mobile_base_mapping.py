#!/usr/bin/env python
"""

"""

import rospy
import math
import numpy as np

from geometry_msgs.msg import (Twist)

from oculus_ros.msg import (
    ControllerButtons,
    ControllerJoystick,
)


class OculusMobileBaseMapping:
    """
    
    """

    def __init__(
        self,
        controller_side='left',
        max_linear_speed=0.5,  # Meters/second.
        max_rotation_speed=60,  # Degrees/seconds.
        linear_acceleration=5,  # Meters/second^2.
        rotation_acceleration=600  # Degrees/second^2.
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.CONTROLLER_SIDE = controller_side

        self.MAX_LINEAR_SPEED = max_linear_speed
        self.MAX_ROTATION_SPEED = math.radians(max_rotation_speed)
        self.LINEAR_ACCELERATION = linear_acceleration
        self.ROTATION_ACCELERATION = math.radians(rotation_acceleration)

        # # Private variables:
        self.__oculus_joystick = ControllerJoystick()
        self.__oculus_buttons = ControllerButtons()

        # Set target velocities.
        self.__target_linear_velocity = 0.0
        self.__target_rotation_velocity = 0.0

        # Calculated velocities based on the acceleration algorithm.
        self.__current_linear_velocity = 0.0
        self.__current_rotation_velocity = 0.0

        self.__previous_time = rospy.get_time()

        self.__joystick_button_state = 0
        self.__control_mode = 'full'

        # # Public variables:

        # # ROS node:

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__mobilebase_twist = rospy.Publisher(
            'base_controller/command',
            Twist,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/joystick',
            ControllerJoystick,
            self.__oculus_joystick_callback,
        )
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick = message

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

    # # Private methods:
    def __check_dead_zones(self):
        """
        
        """

        updated_joystick = np.array(
            [
                self.__oculus_joystick.position_x,
                self.__oculus_joystick.position_y,
            ]
        )

        dead_zone_margins = {
            'up': 15,
            'down': 25,
            'left_right': 0,
        }

        angle = math.degrees(
            math.atan2(updated_joystick[1], updated_joystick[0])
        )

        if (
            (
                angle < 90 + dead_zone_margins['up']
                and angle > 90 - dead_zone_margins['up']
            ) or (
                angle > -90 - dead_zone_margins['down']
                and angle < -90 + dead_zone_margins['down']
            )
        ):
            updated_joystick[0] = 0.0

        elif (
            (
                angle < 0 + dead_zone_margins['left_right']
                and angle > 0 - dead_zone_margins['down']
            ) or (
                angle < -180 + dead_zone_margins['left_right']
                and angle > 180 - dead_zone_margins['down']
            )
        ):
            updated_joystick[1] = 0.0

        return updated_joystick

    def __set_target_velocities(self):
        """
        
        """

        self.__target_linear_velocity = 0.0
        self.__target_rotation_velocity = 0.0

        updated_joystick = self.__check_dead_zones()

        # Linear velocity.
        if abs(self.__oculus_joystick.position_y) > 0.01:  # Noisy joystick.
            self.__target_linear_velocity = np.interp(
                round(updated_joystick[1], 4),
                [-1.0, 1.0],
                [-0.5 * self.MAX_LINEAR_SPEED, self.MAX_LINEAR_SPEED],
            )

        # Rotation velocity.
        if abs(self.__oculus_joystick.position_x) > 0.01:  # Noisy joystick.
            self.__target_rotation_velocity = np.interp(
                round(updated_joystick[0], 4),
                [-1.0, 1.0],
                [self.MAX_ROTATION_SPEED, -self.MAX_ROTATION_SPEED],
            )

    def __update_velocity(
        self,
        target_velocity,
        current_velocity,
        acceleration,
    ):
        """
        
        """

        # Get updated time step.
        current_time = rospy.get_time()
        time_step = current_time - self.__previous_time

        self.__previous_time = current_time

        # Forward motion.
        if target_velocity > 0:
            # Target velocity reached.
            if current_velocity >= target_velocity:
                current_velocity = target_velocity

            # Acceleration.
            else:
                current_velocity = current_velocity + acceleration * time_step

        # Backward motion.
        elif target_velocity < 0:
            # Target velocity reached.
            if current_velocity <= target_velocity:
                current_velocity = target_velocity

            # Acceleration.
            else:
                current_velocity = current_velocity - acceleration * time_step

        # Stopping.
        elif target_velocity == 0:
            # Target velocity reached.
            if abs(current_velocity - target_velocity) < 0.05:
                current_velocity = 0.0

            # Decceleration.
            else:
                if current_velocity > 0:
                    current_velocity = (
                        current_velocity - acceleration * time_step
                    )

                elif current_velocity < 0:
                    current_velocity = (
                        current_velocity + acceleration * time_step
                    )

        return current_velocity

    def __joystick_button_state_machine(self):
        """
        
        """

        # State 0: Joystick button was pressed. Rotation only mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 0
        ):
            self.__control_mode = 'rotation'
            self.__joystick_button_state = 1

        # State 1: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 1
        ):
            self.__joystick_button_state = 2

        # State 2: Joystick button was pressed. Normal control mode.
        if (
            self.__oculus_joystick.button and self.__joystick_button_state == 2
        ):
            self.__control_mode = 'full'
            self.__joystick_button_state = 3

        # State 3: Joystick button was released.
        elif (
            not self.__oculus_joystick.button
            and self.__joystick_button_state == 3
        ):
            self.__joystick_button_state = 0

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__set_target_velocities()

        self.__current_linear_velocity = self.__update_velocity(
            self.__target_linear_velocity,
            self.__current_linear_velocity,
            self.LINEAR_ACCELERATION,
        )
        self.__current_rotation_velocity = self.__update_velocity(
            self.__target_rotation_velocity,
            self.__current_rotation_velocity,
            self.ROTATION_ACCELERATION,
        )

        self.publish_twist_command(
            self.__current_linear_velocity,
            self.__current_rotation_velocity,
        )

    def publish_twist_command(self, linear_velocity, rotation_velocity):
        """
        
        """

        twist_message = Twist()
        twist_message.linear.x = linear_velocity
        twist_message.angular.z = rotation_velocity

        self.__joystick_button_state_machine()

        if self.__control_mode == 'rotation':
            twist_message.linear.x = 0.0

        self.__mobilebase_twist.publish(twist_message)


def node_shutdown():
    """
    
    """

    print('\nNode is shutting down...\n')

    # TODO: Stop mobile base motion.

    print('\nNode is shut down.\n')


def main():
    """
    
    """

    # # ROS node:
    rospy.init_node('oculus_mobile_base_mapping')
    rospy.on_shutdown(node_shutdown)

    mobile_base_mapping = OculusMobileBaseMapping(
        controller_side='left',
        max_linear_speed=0.5,
    )

    print('\nOculus-mobile base mapping is ready.\n')

    while not rospy.is_shutdown():
        mobile_base_mapping.main_loop()


if __name__ == '__main__':
    main()