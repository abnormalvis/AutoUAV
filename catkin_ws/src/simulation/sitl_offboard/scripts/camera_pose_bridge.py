#!/usr/bin/env python3
"""Republish the ground-truth camera pose (Odometry) as a PoseStamped.

The iris_vio model's p3d plugin publishes nav_msgs/Odometry on
/camera/pose_odom. EGO's default launch reads its camera pose from the OpenVINS
odometry (pose_type=2 -> ~grid_map/odom), so this node is a validation aid: it
exposes the same pose as /camera/pose (PoseStamped, world frame) so you can
cross-check VIO against ground truth, or switch EGO to pose_type=1 later.
"""
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class CameraPoseBridge:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/camera/pose_odom")
        self.pose_topic = rospy.get_param("~pose_topic", "/camera/pose")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_cb, queue_size=1
        )

    def _odom_cb(self, msg):
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose = msg.pose.pose
        self.pose_pub.publish(out)


def main():
    rospy.init_node("camera_pose_bridge")
    CameraPoseBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
