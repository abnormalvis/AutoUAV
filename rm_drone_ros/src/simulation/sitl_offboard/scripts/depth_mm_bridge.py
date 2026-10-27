#!/usr/bin/env python3
"""Depth encoding compatibility layer.

The iris_vio model already binds libgazebo_ros_openni_kinect.so with
<useDepth16UC1Format>true</useDepth16UC1Format>, so /camera/depth/image_rect_raw
is published natively as mono16 millimeters and EGO consumes it directly. This
node is the fallback for a sensor that emits 32FC1 meters instead: it converts
32FC1 (meters) to 16UC1 (millimeters) and republishes.

Run it only when `rostopic echo -n1 <depth_in>` shows 32FC1; otherwise it is not
needed and should stay disabled (see sitl.launch's `depth_bridge` arg).
"""
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class DepthMmBridge:
    def __init__(self):
        self.depth_in = rospy.get_param("~depth_in", "/camera/depth/image_raw")
        self.depth_out = rospy.get_param("~depth_out", "/camera/depth/image_rect_raw")
        self.scale = rospy.get_param("~scale", 1000.0)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.depth_out, Image, queue_size=1)
        self.sub = rospy.Subscriber(self.depth_in, Image, self._cb, queue_size=1)

    def _cb(self, msg):
        if msg.encoding not in ("32FC1", "16UC1", "mono16"):
            rospy.logwarn("Unexpected depth encoding %s; passthrough", msg.encoding)
            self.pub.publish(msg)
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding == "32FC1":
            img = (np.nan_to_num(img) * self.scale).astype(np.uint16)
        out = self.bridge.cv2_to_imgmsg(img, encoding="mono16")
        out.header = msg.header
        self.pub.publish(out)


def main():
    rospy.init_node("depth_mm_bridge")
    DepthMmBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
