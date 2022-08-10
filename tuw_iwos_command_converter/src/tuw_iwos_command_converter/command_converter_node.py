#!/usr/bin/env python3

import rospy
import tf

from math import sin
from math import pi
from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
from rospy import Publisher
from tuw_geometry_msgs.msg import TwistWithOrientation
from tuw_nav_msgs.msg import Joints, JointsIWS
from typing import List
from typing import Optional
from typing import Union


class CommandConverterNode:

    def __init__(self):
        self.ANGULAR_VELOCITY_THRESHOLD = 0.01
        self.ORIENTATION_THRESHOLD = 0.01
        self.ORIENTATION_CHANGING_SPEED = 0.5  # (rad/s)
        self.WHEEL_DISPLACEMENT: Optional[float] = None
        self.TARGET_REACHED_DISTANCE = 0.01

        self.state_subscriber_topic: str = "/iwos_state_joints_tuw"
        self.command_subscriber_topic: str = "/iwos_cmd_twist"
        self.command_publisher_topic: str = "/iwos_cmd_joints"
        self.command_subscriber: Optional[Subscriber] = None
        self.state_subscriber: Optional[Subscriber] = None
        self.subscribers: List[Optional[Subscriber]] = [self.command_subscriber, self.state_subscriber]
        self.publisher: Optional[Publisher] = None
        self.time_synchronizer: Optional[TimeSynchronizer] = None

    def run(self):
        rospy.init_node("TUW_IWOS_COMMAND_CONVERTER")

        tf_listener = tf.TransformListener()

        translation: [float] = None  # [x,y,z]

        while translation is None:
            try:
                translation, _ = tf_listener.lookupTransform('base_link', 'left_wheel_link',  rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.WHEEL_DISPLACEMENT = 2 * translation[1]

        self.command_subscriber = Subscriber(self.command_subscriber_topic, TwistWithOrientation)
        self.state_subscriber = Subscriber(self.state_subscriber_topic, Joints)
        self.subscribers: List[Optional[Subscriber]] = [self.command_subscriber, self.state_subscriber]

        # states are published with a frequency of 20hz and a maximum time between messages below 0.1 seconds (~0.09s)
        self.time_synchronizer = ApproximateTimeSynchronizer(self.subscribers, 10, slop=0.1)
        self.time_synchronizer.registerCallback(self.callback)

        self.publisher = Publisher(name=self.command_publisher_topic, data_class=JointsIWS, queue_size=10)

        rospy.spin()

    def callback(self, command_message: TwistWithOrientation, state_message: Joints):
        linear_velocity = command_message.twist.linear.x
        angular_velocity = command_message.twist.angular.z
        orientation = command_message.orientation

        target_angle_left = -orientation
        target_angle_right = -orientation

        left_steering_index = state_message.name.index("left_steering")
        right_steering_index = state_message.name.index("right_steering")

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
            d = self.WHEEL_DISPLACEMENT

            target_velocity_left: float = w * ((v / w) - d / 2.0)
            target_velocity_right: float = w * ((v / w) + d / 2.0)

        if abs(current_angle_left - target_angle_left) > self.ORIENTATION_THRESHOLD:
            left_target_distance = sin(target_angle_left) * self.WHEEL_DISPLACEMENT / 2.0
            left_current_distance = sin(current_angle_left) * self.WHEEL_DISPLACEMENT / 2.0
            left_distance_to_go = left_target_distance - left_current_distance

            if abs(left_distance_to_go) > self.TARGET_REACHED_DISTANCE:
                left_velocity_change = self.signum(left_distance_to_go) * 0.1
                target_velocity_left += left_velocity_change

        if abs(current_angle_right - target_angle_right) > self.ORIENTATION_THRESHOLD:
            right_target_distance = sin(target_angle_right + pi) * self.WHEEL_DISPLACEMENT / 2.0
            right_current_distance = sin(current_angle_right + pi) * self.WHEEL_DISPLACEMENT / 2.0
            right_distance_to_go = right_target_distance - right_current_distance

            if abs(right_distance_to_go) > self.TARGET_REACHED_DISTANCE:
                right_velocity_change = self.signum(right_distance_to_go) * 0.1
                target_velocity_right += right_velocity_change

        output_message = JointsIWS()
        output_message.type_revolute = "cmd_velocity"
        output_message.type_steering = "cmd_position"
        output_message.revolute = [target_velocity_left, target_velocity_right]
        output_message.steering = [target_angle_left, target_angle_right]

        self.publisher.publish(output_message)

    @staticmethod
    def signum(number: Union[int, float]) -> int:
        return -1 if number < 0 else 1


if __name__ == '__main__':
    CommandConverterNode().run()
