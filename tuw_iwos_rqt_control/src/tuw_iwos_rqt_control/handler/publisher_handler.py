#!/usr/bin/env python
import rospy

from tuw_nav_msgs.msg import JointsIWS


class PublisherHandler:

    def __init__(self, plugin, widget):
        self._plugin = plugin
        self._widget = widget
        self._topic_edit = self._widget.topic_edit
        self._topic_edit.textChanged.connect(self._on_topic_change)

        self._topic = None
        self._publisher = None

        self.default()

    def default(self):
        self._topic_edit.setText(rospy.get_param(param_name="iwos_cmd_topic", default="iwos_cmd"))

    def get_topic(self):
        return self._topic

    def get_publisher(self):
        return self._publisher

    def publish(self, message):
        self._publisher.publish(message)

    def _on_topic_change(self):
        """
        sets topic on change
        :return:
        """
        self._topic = self._widget.topic_edit.text()
        self.register_publisher()
        return

    def register_publisher(self):
        """
        registers publisher
        :return:
        """
        self.unregister_publisher()
        if self._topic and not self._topic.isspace() and not self._topic == '/':
            self._publisher = rospy.Publisher(name=self._topic, data_class=JointsIWS, queue_size=10)

    def unregister_publisher(self):
        """
        unregisters publisher
        :return:
        """
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None
