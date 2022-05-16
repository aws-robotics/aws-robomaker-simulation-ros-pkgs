#!/usr/bin/env python

from aws_robomaker_cloudwatch_publisher import CloudWatchPublisher
from aws_robomaker_gazebo_bridge.msg import AwsPerfMetrics
import rospy


class RTFCloudWatchPublisher(object):
    def __init__(self, publish_rate_frequency=1):
        rospy.init_node("RTFCloudWatchPublisher")
        self._freq = rospy.get_param("~publish_rate_frequency")
        self._publisher = CloudWatchPublisher("RealTimeFactor")

        self._rtf = None
        gazebo_topic = rospy.get_param("~gazebo_topic")
        self._gazebo_sub = rospy.Subscriber(gazebo_topic, AwsPerfMetrics, self._on_gazebo_rx)

    def _on_gazebo_rx(self, msg):
        self._rtf = msg.rtf

    def publish(self):
        if self._rtf is None:
            rospy.logwarn(
                "Cannot publish rtf as no message has been received since last publish! "
                "Is the publish frequency too high?"
            )
            return

        rospy.logdebug("Publishing real-time factor: %f", self._rtf)
        self._publisher.put_metric(self._rtf)
        self._rtf = None

    def run(self):
        rate = rospy.Rate(self._freq)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


def main():
    RTFCloudWatchPublisher().run()


if __name__ == "__main__":
    main()
