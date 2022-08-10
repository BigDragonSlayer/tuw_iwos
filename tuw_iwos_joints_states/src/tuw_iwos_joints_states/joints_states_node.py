#!/usr/bin/env python3

import rospy
import sys

from typing import List
from typing import NoReturn
from typing import Optional

from message_filters import ApproximateTimeSynchronizer
from message_filters import Subscriber
from rospy import Publisher
from sensor_msgs.msg import JointState
from tuw_nav_msgs.msg import Joints


class JointsStatesNode:

    def __init__(self, states_topics: List[str]) -> None:
        self._node_name = 'IWOS_JOINTS_STATES'

        self._states_topics: List[str] = states_topics
        self._state_subscribers: Optional[List[Subscriber]] = None
        self._publisher_tuw: Optional[Publisher] = None
        self._publisher: Optional[Publisher] = None

    def run(self) -> NoReturn:
        rospy.init_node(self._node_name)
        self._state_subscribers = [Subscriber(topic, Joints) for topic in self._states_topics]
        self._publisher_tuw = Publisher(name="iwos_state_joints_tuw", data_class=Joints, queue_size=100)
        self._publisher = Publisher(name="iwos_state_joints", data_class=JointState, queue_size=100)
        slop: float = float(1 / (2 * rospy.get_param(param_name='joint_state_hz', default=30)))
        time_synchronizer = ApproximateTimeSynchronizer(fs=self._state_subscribers, queue_size=100, slop=slop)
        time_synchronizer.registerCallback(cb=self.time_synchronizer_callback)

        rospy.spin()

    def time_synchronizer_callback(self, *messages: Joints) -> None:
        header = messages[0].header
        name = [n for n_list in [list(message.name) for message in messages] for n in n_list]
        position = [p for p_list in [list(message.position) for message in messages] for p in p_list]
        velocity = [v for v_list in [list(message.velocity) for message in messages] for v in v_list]
        torque = [t for t_list in [list(message.torque) for message in messages] for t in t_list]
        effort = torque

        state_tuw = Joints(header=header, name=name, position=position, velocity=velocity, torque=torque)
        state = JointState(header=header, name=name, position=position, velocity=velocity, effort=effort)

        self._publisher_tuw.publish(state_tuw)
        self._publisher.publish(state)


if __name__ == '__main__':
    try:
        arguments = rospy.myargv(argv=sys.argv)
        states_topics_arg = arguments[1:]
        steering_controller_node = JointsStatesNode(states_topics=states_topics_arg)
        steering_controller_node.run()
    except rospy.ROSInterruptException:
        rospy.logerr('ROS Interrupt Exception')
