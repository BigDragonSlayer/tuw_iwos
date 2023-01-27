#!/usr/bin/env python3

import math
import rospy
import tf

from math import sin
from math import pi
from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
from rospy import Publisher
from tuw_geometry_msgs.msg import TwistWithOrientation
from tuw_nav_msgs.msg import Joints, JointsIWS
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from typing import List
from typing import Optional
from typing import Union


class CommandConverterNode:

    def __init__(self):
        self.ANGULAR_VELOCITY_THRESHOLD = 0.1
        self.ORIENTATION_THRESHOLD = 0.05
        self.TARGET_REACHED_DISTANCE = 0.05
        self.ORIENTATION_CHANGING_SPEED = 0.5  # (rad/s)
        self.wheel_displacement: Optional[float] = 0.4

        self.state_subscriber_topic: str = "/joint_state"
        self.command_subscriber_topic: str = "/iwos_command_twist_with_orientation"
        self.command_publisher_topic: str = "/iwos_command_hardware"
        self.command_subscriber: Optional[Subscriber] = None
        self.state_subscriber: Optional[Subscriber] = None
        self.subscribers: List[Optional[Subscriber]] = [self.command_subscriber, self.state_subscriber]
        self.publisher: Optional[Publisher] = None
        self.time_synchronizer: Optional[TimeSynchronizer] = None

    def run(self):
        rospy.init_node("TUW_IWOS_COMMAND_CONVERTER")

        self.command_subscriber = Subscriber(self.command_subscriber_topic, TwistWithOrientation)
        self.state_subscriber = Subscriber(self.state_subscriber_topic, JointState)
        self.subscribers: List[Optional[Subscriber]] = [self.command_subscriber, self.state_subscriber]

        # states are published with a frequency of 20hz and a maximum time between messages below 0.1 seconds (~0.09s)
        self.time_synchronizer = ApproximateTimeSynchronizer(self.subscribers, 10, slop=0.1)
        self.time_synchronizer.registerCallback(self.callback)

        self.publisher = Publisher(name=self.command_publisher_topic, data_class=JointsIWS, queue_size=10)

        rospy.spin()

    def callback(self, command_message: TwistWithOrientation, state_message: Joints):
        self.wheel_displacement = self.lookup_wheel_displacement()
        linear_velocity = command_message.twist.linear.x
        angular_velocity = command_message.twist.angular.z
        orientation = command_message.orientation

        target_angle_left = -orientation
        target_angle_right = -orientation

        left_steering_index = state_message.name.index("steering_left")
        right_steering_index = state_message.name.index("steering_right")

        current_angle_left = state_message.position[left_steering_index]
        current_angle_right = state_message.position[right_steering_index]

        target_velocity_left: Optional[float] = None
        target_velocity_right: Optional[float] = None

        if abs(angular_velocity) < self.ANGULAR_VELOCITY_THRESHOLD:
            target_velocity_left = linear_velocity
            target_velocity_right = linear_velocity

        if not abs(angular_velocity) < self.ANGULAR_VELOCITY_THRESHOLD:
            v = linear_velocity
            w = angular_velocity
            b = self.wheel_displacement

            target_velocity_left: float = w * ((v / w) - b / 2.0)
            target_velocity_right: float = w * ((v / w) + b / 2.0)

        if abs(current_angle_left - target_angle_left) > self.ORIENTATION_THRESHOLD:
            left_target_distance = sin(target_angle_left) * self.wheel_displacement / 2.0
            left_current_distance = sin(current_angle_left) * self.wheel_displacement / 2.0
            left_distance_to_go = left_target_distance - left_current_distance

            if abs(left_distance_to_go) > self.TARGET_REACHED_DISTANCE:
                left_velocity_change = self.signum(left_distance_to_go) * 0.1
                target_velocity_left += left_velocity_change

        if abs(current_angle_right - target_angle_right) > self.ORIENTATION_THRESHOLD:
            right_target_distance = sin(target_angle_right + pi) * self.wheel_displacement / 2.0
            right_current_distance = sin(current_angle_right + pi) * self.wheel_displacement / 2.0
            right_distance_to_go = right_target_distance - right_current_distance

            if abs(right_distance_to_go) > self.TARGET_REACHED_DISTANCE:
                right_velocity_change = self.signum(right_distance_to_go) * 0.1
                target_velocity_right += right_velocity_change

        if abs(target_velocity_left) > 1.0 or abs(target_velocity_right) > 1.0:
            maximum = max(abs(target_velocity_left), abs(target_velocity_right))
            target_velocity_left = target_velocity_left/abs(maximum)
            target_velocity_right = target_velocity_right/abs(maximum)

        output_message_header = Header()
        output_message_header.seq = command_message.header.seq
        output_message_header.stamp = command_message.header.stamp
        output_message_header.frame_id = "base_link"

        output_message = JointsIWS()
        output_message.header = output_message_header
        output_message.type_revolute = "cmd_velocity"
        output_message.type_steering = "cmd_position"
        output_message.revolute = [target_velocity_left, target_velocity_right]
        output_message.steering = [target_angle_left, target_angle_right]

        self.publisher.publish(output_message)

    def lookup_wheel_displacement(self) -> float:
        tf_listener = tf.TransformListener()

        translation = {"left": None, "right": None}  # {[x,y,z]}

        # for side, value in translation.items():
        #     target_link = "wheel_link_" + side
        #     try:
        #         value, _ = tf_listener.lookupTransform('base_link', target_link,  rospy.Time(0))
        #         if value is not None:
        #             translation[side] = value
        #         if value is None:
        #             rospy.log_debug("failed to fetch current wheel displacement")
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         rospy.logdebug("failed to fetch current wheel displacement")
        #         continue

        if translation["left"] is not None and translation["right"] is not None:
            return translation["left"][1] - translation["right"][1]
        else:
            return self.wheel_displacement

    @staticmethod
    def signum(number: Union[int, float]) -> int:
        return -1 if number < 0 else 1


if __name__ == '__main__':
    CommandConverterNode().run()
