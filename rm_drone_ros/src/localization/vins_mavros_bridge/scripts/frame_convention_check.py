#!/usr/bin/env python3
"""End-to-end check of the bridge's twist frame handling.

The pose stream cannot reveal a twist-frame mistake: the position is identical
whichever frame the velocity was in, so a wrong adapter looks like good
localization with unexplained hover drift. These scenarios therefore assert on
the *velocity* that comes out, against a velocity that was constructed in a
known frame on the way in.

Three scenarios, each matching one real adapter:

  openvins      Odometry, body-frame linear velocity        -> must be rotated
  vins_fusion   Odometry, world-frame linear, no angular    -> must pass through
  derived       PoseWithCovarianceStamped, no twist at all  -> must be estimated

The script only publishes and asserts; the caller starts the bridge with the
matching parameters.
"""

import argparse
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class Collector(object):
    def __init__(self):
        self.samples = []
        self.lock = threading.Lock()
        self.sub = rospy.Subscriber("/localization/odometry", Odometry, self._callback, queue_size=50)

    def _callback(self, msg):
        with self.lock:
            self.samples.append(msg)

    def snapshot(self):
        with self.lock:
            return list(self.samples)

    def clear(self):
        with self.lock:
            self.samples = []

    def latest_after(self, count):
        """Returns the last `count` samples once that many have accumulated."""
        deadline = time.time() + 10.0
        while time.time() < deadline:
            samples = self.snapshot()
            if len(samples) >= count:
                return samples[-count:]
            time.sleep(0.05)
        return []


def publish_odometry(pub, yaw, linear_body_or_world, child_frame, angular=None, duration=2.0):
    rate = rospy.Rate(30.0)
    end = time.time() + duration
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    while not rospy.is_shutdown() and time.time() < end:
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.child_frame_id = child_frame
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = linear_body_or_world[0]
        msg.twist.twist.linear.y = linear_body_or_world[1]
        msg.twist.twist.linear.z = linear_body_or_world[2]
        if angular is not None:
            msg.twist.twist.angular.x = angular[0]
            msg.twist.twist.angular.y = angular[1]
            msg.twist.twist.angular.z = angular[2]
        pub.publish(msg)
        rate.sleep()


def publish_moving_pose(pub, yaw0, yaw_rate, velocity_world, duration=2.5):
    """Constant world velocity plus constant world yaw rate, starting at t=0."""
    rate = rospy.Rate(30.0)
    start = rospy.Time.now()
    end = start.to_sec() + duration
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() < end:
        t = (rospy.Time.now() - start).to_sec()
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.pose.position.x = velocity_world[0] * t
        msg.pose.pose.position.y = velocity_world[1] * t
        msg.pose.pose.position.z = velocity_world[2] * t
        q = quaternion_from_yaw(yaw0 + yaw_rate * t)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.pose.covariance[0] = 0.0025
        msg.pose.covariance[7] = 0.0025
        msg.pose.covariance[14] = 0.0025
        pub.publish(msg)
        rate.sleep()


def twist_of(msg):
    linear = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
    angular = (msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z)
    return linear, angular


def close(actual, expected, tolerance):
    return all(abs(a - e) <= tolerance for a, e in zip(actual, expected))


def run_openvins(collector):
    """Body-frame velocity at yaw = 90 deg must come out rotated into world."""
    pub = rospy.Publisher("/check/odom", Odometry, queue_size=10)
    time.sleep(0.3)
    collector.clear()
    publisher = threading.Thread(
        target=publish_odometry,
        args=(pub, math.pi / 2.0, (1.0, 0.0, 0.0), "imu"),
        kwargs={"duration": 3.0},
    )
    publisher.start()
    samples = collector.latest_after(10)
    publisher.join()
    if not samples:
        print("FAIL openvins: no output on /localization/odometry")
        return False
    linear, _ = twist_of(samples[-1])
    ok = close(linear, (0.0, 1.0, 0.0), 1e-6)
    print("%s openvins: body twist (1,0,0) at yaw 90 -> world %s, expected (0,1,0)"
          % ("PASS" if ok else "FAIL", linear))
    return ok


def run_vins_fusion(collector):
    """World-frame velocity must pass through untouched, angular must stay zero."""
    pub = rospy.Publisher("/check/odom", Odometry, queue_size=10)
    time.sleep(0.3)
    collector.clear()
    # VINS-Fusion publishes child_frame_id "world" on an IMU-frame pose, and
    # leaves the angular rate at zero.
    publisher = threading.Thread(
        target=publish_odometry,
        args=(pub, math.pi / 2.0, (1.0, 0.0, 0.0), "world", (9.0, 9.0, 9.0)),
        kwargs={"duration": 3.0},
    )
    publisher.start()
    samples = collector.latest_after(10)
    publisher.join()
    if not samples:
        print("FAIL vins_fusion: no output on /localization/odometry")
        return False
    linear, angular = twist_of(samples[-1])
    ok_linear = close(linear, (1.0, 0.0, 0.0), 1e-6)
    # input_twist_angular: false must suppress the bogus angular rate.
    ok_angular = close(angular, (0.0, 0.0, 0.0), 1e-9)
    print("%s vins_fusion: world twist (1,0,0) at yaw 90 -> %s (expected (1,0,0)); "
          "angular %s (expected zero)"
          % ("PASS" if (ok_linear and ok_angular) else "FAIL", linear, angular))
    return ok_linear and ok_angular


def run_derived(collector):
    """A pose-only source must still yield a usable world-frame velocity."""
    pub = rospy.Publisher("/check/pose", PoseWithCovarianceStamped, queue_size=10)
    time.sleep(0.3)
    collector.clear()
    # 0.5 m/s along world X while yawing at 0.5 rad/s about world Z. Because the
    # body's Z axis stays aligned with world Z, the body-frame yaw rate equals
    # the world-frame one, so this checks the magnitude and sign, not the frame.
    publisher = threading.Thread(
        target=publish_moving_pose,
        args=(pub, 0.0, 0.5, (0.5, 0.0, 0.0)),
        kwargs={"duration": 3.0},
    )
    publisher.start()
    samples = collector.latest_after(10)
    publisher.join()
    if not samples:
        print("FAIL derived: no output on /localization/odometry")
        return False
    linear, angular = twist_of(samples[-1])
    ok_linear = close(linear, (0.5, 0.0, 0.0), 0.1)
    ok_angular = close(angular, (0.0, 0.0, 0.5), 0.15)
    print("%s derived: 0.5 m/s + 0.5 rad/s -> linear %s (expected (0.5,0,0)), "
          "angular %s (expected (0,0,0.5))"
          % ("PASS" if (ok_linear and ok_angular) else "FAIL", linear, angular))
    return ok_linear and ok_angular


SCENARIOS = {
    "openvins": run_openvins,
    "vins_fusion": run_vins_fusion,
    "derived": run_derived,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", choices=sorted(SCENARIOS))
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("frame_convention_check", anonymous=True)
    collector = Collector()
    time.sleep(0.5)
    ok = SCENARIOS[args.scenario](collector)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
