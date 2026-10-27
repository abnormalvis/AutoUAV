#!/usr/bin/env python3
import math
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class KeyboardGoal:
    def __init__(self):
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.odom_topic = rospy.get_param("~odom_topic", "/ov_msckf/odomimu")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.step = rospy.get_param("~step", 0.5)
        self.max_radius = rospy.get_param("~max_radius", 8.0)
        self.odom_timeout = rospy.get_param("~odom_timeout", 0.5)
        self.origin = None
        self.goal = None
        self.last_odom_time = None
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)

    def _odom_cb(self, msg):
        self._latest_odom = msg
        self.last_odom_time = rospy.Time.now()
        position = msg.pose.pose.position
        if self.origin is None:
            self.origin = (position.x, position.y, position.z)
            self.goal = [position.x, position.y, position.z]

    def _move(self, dx=0.0, dy=0.0):
        if self.goal is None:
            return
        x = self.goal[0] + dx
        y = self.goal[1] + dy
        offset_x = x - self.origin[0]
        offset_y = y - self.origin[1]
        distance = math.hypot(offset_x, offset_y)
        if distance > self.max_radius:
            scale = self.max_radius / distance
            x = self.origin[0] + offset_x * scale
            y = self.origin[1] + offset_y * scale
        self.goal = [x, y, self.goal[2]]
        rospy.loginfo("Target [world]: x=%.2f y=%.2f", x, y)

    def publish(self):
        if self.goal is None or self.last_odom_time is None:
            rospy.logwarn("No odometry received; target not published")
            return
        if (rospy.Time.now() - self.last_odom_time).to_sec() > self.odom_timeout:
            rospy.logwarn("Odometry is stale; target not published")
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.goal
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo("Published goal to %s", self.goal_topic)

    def run(self):
        if not sys.stdin.isatty():
            raise RuntimeError("Keyboard goal control requires an interactive terminal")

        rospy.loginfo(
            "Keyboard goal control: w/s x, a/d y, Enter publish, "
            "space reset to current odometry, Ctrl-C quit"
        )
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while not rospy.is_shutdown():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key == "\x03":
                    break
                if key in ("\r", "\n"):
                    self.publish()
                elif key == " ":
                    self._reset_to_latest()
                elif key == "w":
                    self._move(dx=self.step)
                elif key == "s":
                    self._move(dx=-self.step)
                elif key == "a":
                    self._move(dy=self.step)
                elif key == "d":
                    self._move(dy=-self.step)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def _reset_to_latest(self):
        if self.goal is None or not hasattr(self, "_latest_odom"):
            rospy.logwarn("No odometry received; cannot reset target")
            return
        msg = self._latest_odom
        position = msg.pose.pose.position
        self.goal = [position.x, position.y, position.z]
        rospy.loginfo("Target reset to current VIO pose")


def main():
    rospy.init_node("keyboard_goal")
    controller = KeyboardGoal()
    try:
        controller.run()
    except RuntimeError as exc:
        rospy.logfatal(str(exc))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())