#!/usr/bin/env python3
"""Relay iris_vio Gazebo ground truth to Prometheus' expected odometry topic."""

import rospy
from nav_msgs.msg import Odometry


class PrometheusTruthBridge:
    def __init__(self):
        uav_id = rospy.get_param("~uav_id", 1)
        input_topic = rospy.get_param("~input_topic", "/camera/pose_odom")
        output_topic = rospy.get_param(
            "~output_topic", f"/uav{uav_id}/prometheus/ground_truth"
        )
        self.uav_id = uav_id
        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, Odometry, self._callback, queue_size=1
        )
        rospy.loginfo("Prometheus truth bridge: %s -> %s", input_topic, output_topic)

    def _callback(self, message):
        output = Odometry()
        output.header = message.header
        if not output.header.frame_id:
            output.header.frame_id = "world"
        output.child_frame_id = f"uav{self.uav_id}/base_link"
        output.pose = message.pose
        output.twist = message.twist
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("prometheus_truth_bridge")
    PrometheusTruthBridge()
    rospy.spin()