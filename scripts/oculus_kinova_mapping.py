#!/usr/bin/env python
"""

"""

import rospy

from geometry_msgs.msg import (Pose)
from std_msgs.msg import (Bool)

from oculus.msg import (ControllerButtons)
from kinova_positional_control.srv import (
    GripperForceGrasping,
    GripperPosition,
)


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
        self.__gripper_state_machine_state = 0
        self.__mode_state_machine_state = 0
        self.__control_mode = 'position'

        # # Public variables:
        self.tracking = False

        # # ROS node:

        # # Service provider:

        # # Service subscriber:
        self.__gripper_force_grasping = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/gripper/force_grasping',
            GripperForceGrasping,
        )
        self.__gripper_position = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/gripper/position',
            GripperPosition,
        )

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

        if self.__control_mode == 'full':
            self.__oculus_pose = message

        elif self.__control_mode == 'position':
            self.__oculus_pose.position = message.position
            # TODO: Proper orientation support.
            self.__oculus_pose.orientation.w = 0.6532815
            self.__oculus_pose.orientation.x = -0.2705981
            self.__oculus_pose.orientation.y = -0.2705981
            self.__oculus_pose.orientation.z = 0.6532815

        elif self.__control_mode == 'orientation':
            self.__oculus_pose.orientation = message.orientation

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

        if self.TRACKING_MODE == 'hold':
            self.tracking = self.__oculus_buttons.grip_button

    # # Private methods:
    def __tracking_state_machine(self, button):
        """
        
        """

        # State 0: Grip button was pressed.
        if (self.__tracking_state_machine_state == 0 and button):
            self.__tracking_state_machine_state = 1

        # State 1: Grip button was released. Tracking is activated.
        elif (self.__tracking_state_machine_state == 1 and not button):
            self.tracking = True
            self.__tracking_state_machine_state = 2

        # State 2: Grip button was pressed. Tracking is deactivated.
        elif (self.__tracking_state_machine_state == 2 and button):
            self.tracking = False
            self.__tracking_state_machine_state = 3

        # State 3: Grip button was released.
        elif (self.__tracking_state_machine_state == 3 and not button):
            self.__tracking_state_machine_state = 0

    def __gripper_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__gripper_state_machine_state == 0 and button):
            self.__gripper_force_grasping(0.0)  # 0.0 for default current.
            self.__gripper_state_machine_state = 1

        # State 1: Button was released. Force grasping is activated.
        elif (self.__gripper_state_machine_state == 1 and not button):
            self.__gripper_state_machine_state = 2

        # State 2: Button was pressed. Open the gripper.
        elif (self.__gripper_state_machine_state == 2 and button):
            self.__gripper_position(0.0)  # 0.0 for open position.
            self.__gripper_state_machine_state = 3

        # State 3: Button was released.
        elif (self.__gripper_state_machine_state == 3 and not button):
            self.__gripper_state_machine_state = 0

    def __mode_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__mode_state_machine_state == 0 and button):
            self.__control_mode = 'position'
            self.__mode_state_machine_state = 1

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):
            self.__mode_state_machine_state = 3

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 3 and button):
            self.__control_mode = 'full'
            self.__mode_state_machine_state = 4

        # State 3: Button was released.
        elif (self.__mode_state_machine_state == 4 and not button):
            self.__mode_state_machine_state = 0

    # # Public methods:
    def main_loop(self):
        """
        
        """

        if self.TRACKING_MODE == 'press':
            self.__tracking_state_machine(self.__oculus_buttons.grip_button)

        self.__kinova_tracking.publish(self.tracking)
        self.__kinova_pose.publish(self.__oculus_pose)

        self.__gripper_state_machine(self.__oculus_buttons.trigger_button)
        self.__mode_state_machine(self.__oculus_buttons.primary_button)


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
