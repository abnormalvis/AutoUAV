#!/usr/bin/env python3
import math

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PlayTuneV2, State
from nav_msgs.msg import Odometry


class ForwardOffboardMove:
    WAIT_OFFBOARD = "WAIT_OFFBOARD"
    START_DELAY = "START_DELAY"
    MOVING = "MOVING"
    DONE = "DONE"
    ABORTED = "ABORTED"

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/mavros/local_position/odom")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/mavros/setpoint_position/local"
        )
        self.play_tune_topic = rospy.get_param("~play_tune_topic", "/mavros/play_tune")

        self.forward_distance = float(rospy.get_param("~forward_distance", 0.30))
        self.forward_speed = float(rospy.get_param("~forward_speed", 0.08))
        self.start_delay = float(rospy.get_param("~start_delay", 1.0))
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.05))
        self.setpoint_frame_id = rospy.get_param("~setpoint_frame_id", "")

        self.moving_long_tune = rospy.get_param("~moving_long_tune", "MFT120L4O5C")
        self.moving_short_tune = rospy.get_param("~moving_short_tune", "MFT240L16O5C")
        self.done_tune = rospy.get_param("~done_tune", "MFT240L16O5C")
        self.moving_tune_interval = float(rospy.get_param("~moving_tune_interval", 1.0))
        self.done_tune_interval = float(rospy.get_param("~done_tune_interval", 0.45))
        self.enable_buzzer = bool(rospy.get_param("~enable_buzzer", True))

        if self.forward_distance <= 0.0:
            raise ValueError("~forward_distance must be positive")
        if self.forward_speed <= 0.0:
            raise ValueError("~forward_speed must be positive")
        if self.rate_hz < 2.0:
            raise ValueError("~rate_hz must be at least 2.0 for PX4 Offboard")

        self.current_state = State()
        self.current_odom = None
        self.last_hold_pose = None

        self.phase = self.WAIT_OFFBOARD
        self.start_time = None
        self.move_start_time = None
        self.start_position = None
        self.target_position = None
        self.forward_direction = None
        self.start_yaw = 0.0

        self.next_moving_tune_time = rospy.Time(0)
        self.next_done_tune_time = rospy.Time(0)
        self.next_tune_is_long = True
        self.done_tune_count = 0
        self.target_setpoint_reached = False

        self.setpoint_pub = rospy.Publisher(
            self.setpoint_topic, PoseStamped, queue_size=10
        )
        self.tune_pub = rospy.Publisher(self.play_tune_topic, PlayTuneV2, queue_size=3)

        self.state_sub = rospy.Subscriber(
            self.state_topic, State, self._state_cb, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_cb, queue_size=20
        )

        rospy.loginfo(
            "simple_offboard_forward ready: odom=%s setpoint=%s distance=%.3fm speed=%.3fm/s delay=%.2fs",
            self.odom_topic,
            self.setpoint_topic,
            self.forward_distance,
            self.forward_speed,
            self.start_delay,
        )
        rospy.loginfo("Start this node before switching to OFFBOARD.")

    def _state_cb(self, msg):
        self.current_state = msg

    def _odom_cb(self, msg):
        self.current_odom = msg
        if self.phase in (self.WAIT_OFFBOARD, self.ABORTED):
            self.last_hold_pose = self._pose_from_odom(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._update_phase(now)
            self._publish_setpoint(now)
            self._publish_buzzer(now)
            rate.sleep()

    def _update_phase(self, now):
        if self.current_odom is None:
            rospy.logwarn_throttle(2.0, "Waiting for odometry: %s", self.odom_topic)
            return

        is_offboard = self.current_state.mode == "OFFBOARD"

        if self.phase == self.WAIT_OFFBOARD:
            if is_offboard:
                self._start_delay_phase(now)
            return

        if self.phase in (self.START_DELAY, self.MOVING) and not is_offboard:
            self.phase = self.ABORTED
            self.last_hold_pose = self._pose_from_odom(self.current_odom)
            rospy.logwarn("OFFBOARD exited before move completed. Task aborted.")
            return

        if self.phase == self.START_DELAY:
            if (now - self.start_time).to_sec() >= self.start_delay:
                self.phase = self.MOVING
                self.move_start_time = now
                self.next_moving_tune_time = now
                self.next_tune_is_long = True
                rospy.loginfo(
                    "Starting forward move: distance=%.3fm speed=%.3fm/s",
                    self.forward_distance,
                    self.forward_speed,
                )
            return

        if self.phase == self.MOVING:
            traveled = self._planned_distance(now)
            if traveled >= self.forward_distance:
                self.target_setpoint_reached = True

            if self.target_setpoint_reached and self._actual_target_reached():
                self.phase = self.DONE
                self.done_tune_count = 0
                self.next_done_tune_time = now
                rospy.loginfo(
                    "Forward move target reached by odometry. Holding target setpoint."
                )
            elif self.target_setpoint_reached:
                rospy.loginfo_throttle(
                    1.0,
                    "Target setpoint is 0.30m ahead; waiting for vehicle to reach it. position_error=%.3fm",
                    self._target_position_error(),
                )

    def _start_delay_phase(self, now):
        self.start_time = now
        self.start_position = self._position_from_odom(self.current_odom)
        self.start_yaw = self._yaw_from_odom(self.current_odom)
        self.forward_direction = (math.cos(self.start_yaw), math.sin(self.start_yaw), 0.0)
        self.target_position = (
            self.start_position[0] + self.forward_direction[0] * self.forward_distance,
            self.start_position[1] + self.forward_direction[1] * self.forward_distance,
            self.start_position[2],
        )
        self.target_setpoint_reached = False
        self.phase = self.START_DELAY
        rospy.loginfo(
            "OFFBOARD detected. Holding for %.2fs before moving. start=(%.3f, %.3f, %.3f), yaw=%.1fdeg, target=(%.3f, %.3f, %.3f)",
            self.start_delay,
            self.start_position[0],
            self.start_position[1],
            self.start_position[2],
            math.degrees(self.start_yaw),
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
        )

    def _publish_setpoint(self, now):
        if self.current_odom is None:
            return

        if self.phase == self.WAIT_OFFBOARD:
            pose = self._pose_from_odom(self.current_odom)
        elif self.phase == self.START_DELAY:
            pose = self._pose_from_xyz_yaw(self.start_position, self.start_yaw)
        elif self.phase == self.MOVING:
            traveled = self._planned_distance(now)
            position = (
                self.start_position[0] + self.forward_direction[0] * traveled,
                self.start_position[1] + self.forward_direction[1] * traveled,
                self.start_position[2],
            )
            pose = self._pose_from_xyz_yaw(position, self.start_yaw)
        elif self.phase == self.DONE:
            pose = self._pose_from_xyz_yaw(self.target_position, self.start_yaw)
        else:
            pose = self.last_hold_pose or self._pose_from_odom(self.current_odom)

        pose.header.stamp = now
        self.setpoint_pub.publish(pose)

    def _publish_buzzer(self, now):
        if not self.enable_buzzer:
            return

        if self.phase == self.MOVING and now >= self.next_moving_tune_time:
            tune = self.moving_long_tune if self.next_tune_is_long else self.moving_short_tune
            self._publish_tune(tune)
            self.next_tune_is_long = not self.next_tune_is_long
            self.next_moving_tune_time = now + rospy.Duration(self.moving_tune_interval)
            return

        if (
            self.phase == self.DONE
            and self.done_tune_count < 3
            and now >= self.next_done_tune_time
        ):
            self._publish_tune(self.done_tune)
            self.done_tune_count += 1
            self.next_done_tune_time = now + rospy.Duration(self.done_tune_interval)

    def _publish_tune(self, tune):
        msg = PlayTuneV2()
        msg.format = 1
        msg.tune = tune
        self.tune_pub.publish(msg)

    def _planned_distance(self, now):
        elapsed = (now - self.move_start_time).to_sec()
        return min(self.forward_distance, max(0.0, elapsed * self.forward_speed))

    def _actual_target_reached(self):
        return self._target_position_error() <= self.position_tolerance

    def _target_position_error(self):
        if self.current_odom is None or self.target_position is None:
            return float("inf")
        current = self._position_from_odom(self.current_odom)
        dx = current[0] - self.target_position[0]
        dy = current[1] - self.target_position[1]
        dz = current[2] - self.target_position[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _pose_from_odom(self, odom):
        position = self._position_from_odom(odom)
        yaw = self._yaw_from_odom(odom)
        return self._pose_from_xyz_yaw(position, yaw, odom.header.frame_id)

    def _pose_from_xyz_yaw(self, position, yaw, frame_id=None):
        pose = PoseStamped()
        pose.header.frame_id = self._resolve_frame_id(frame_id)
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def _resolve_frame_id(self, feedback_frame_id):
        if self.setpoint_frame_id:
            return self.setpoint_frame_id
        if feedback_frame_id:
            return feedback_frame_id
        if self.current_odom and self.current_odom.header.frame_id:
            return self.current_odom.header.frame_id
        return "map"

    @staticmethod
    def _position_from_odom(odom):
        p = odom.pose.pose.position
        return (p.x, p.y, p.z)

    @staticmethod
    def _yaw_from_odom(odom):
        q = odom.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        return yaw


def main():
    rospy.init_node("forward_30cm_offboard")
    node = ForwardOffboardMove()
    node.spin()


if __name__ == "__main__":
    main()
