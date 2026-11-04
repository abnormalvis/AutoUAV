#!/usr/bin/env python3
"""Synthetic VIO publisher for exercising vins_mavros_bridge without a camera.

Emits nav_msgs/Odometry with OpenVINS's field semantics (child_frame_id "imu",
twist.linear in the body frame) so the bridge's frame normalization can be
verified without any hardware.

Modes:
  hover      stationary at a fixed pose, zero velocity
  yaw90      yaw = 90 deg with body-frame velocity (1, 0, 0); the bridge must
             publish world-frame (0, 1, 0) when input_twist_frame is "body",
             and pass (1, 0, 0) through unchanged when it is "world"
  translate  slow constant world-frame motion, body velocity rotated to match
  jump       a 5 m position step after 3 s, to trigger jump rejection
  divergent  runaway position and velocity, matching the known-bad VIO bag
  silent     publishes for 2 s and then stops, to trigger the staleness path
"""

import argparse
import math

import rospy
from nav_msgs.msg import Odometry

FIELDS = "header, child_frame_id, pose, twist"


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def make_msg(stamp, position, yaw, body_velocity, frame_id, child_frame_id):
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.child_frame_id = child_frame_id

    msg.pose.pose.position.x = position[0]
    msg.pose.pose.position.y = position[1]
    msg.pose.pose.position.z = position[2]

    qx, qy, qz, qw = quat_from_yaw(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    # OpenVINS leaves the angular rates as placeholders; the bridge is told to
    # ignore them via input_twist_angular: false.
    msg.twist.twist.linear.x = body_velocity[0]
    msg.twist.twist.linear.y = body_velocity[1]
    msg.twist.twist.linear.z = body_velocity[2]

    for i in range(36):
        msg.pose.covariance[i] = 0.0
        msg.twist.covariance[i] = 0.0
    for i in range(6):
        msg.pose.covariance[i * 7] = 1e-3
        msg.twist.covariance[i * 7] = 1e-3
    return msg


def body_from_world(velocity_world, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy, vz = velocity_world
    return (c * vx + s * vy, -s * vx + c * vy, vz)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/ov_msckf/odomimu")
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--frame-id", default="global")
    parser.add_argument("--child-frame-id", default="imu")
    parser.add_argument(
        "--mode",
        default="yaw90",
        choices=["hover", "yaw90", "translate", "jump", "divergent", "silent"],
    )
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds; 0 runs until interrupted")
    args = parser.parse_args()

    rospy.init_node("synthetic_odom_publisher", anonymous=True)
    pub = rospy.Publisher(args.topic, Odometry, queue_size=20)
    rate = rospy.Rate(args.rate)

    start = rospy.Time.now()
    rospy.loginfo("synthetic_odom_publisher: mode=%s topic=%s rate=%.1f Hz",
                  args.mode, args.topic, args.rate)

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        if args.duration > 0.0 and elapsed > args.duration:
            break
        if args.mode == "silent" and elapsed > 2.0:
            rospy.loginfo("synthetic_odom_publisher: silent from t=%.1f s", elapsed)
            rate.sleep()
            continue

        if args.mode in ("hover", "silent"):
            position, yaw, body_velocity = (1.0, 2.0, 1.5), 0.0, (0.0, 0.0, 0.0)
        elif args.mode == "yaw90":
            position, yaw, body_velocity = (0.0, 0.0, 1.0), math.pi / 2.0, (1.0, 0.0, 0.0)
        elif args.mode == "translate":
            yaw = 0.3
            velocity_world = (0.5, 0.2, 0.0)
            position = (velocity_world[0] * elapsed, velocity_world[1] * elapsed, 1.0)
            body_velocity = body_from_world(velocity_world, yaw)
        elif args.mode == "jump":
            position = (5.0 if elapsed > 3.0 else 0.0, 0.0, 1.0)
            yaw, body_velocity = 0.0, (0.0, 0.0, 0.0)
        else:  # divergent
            position = (-15095.0 * elapsed, 0.0, 0.0)
            yaw, body_velocity = 0.0, (456.0, 0.0, 0.0)

        pub.publish(make_msg(rospy.Time.now(), position, yaw, body_velocity,
                             args.frame_id, args.child_frame_id))
        rate.sleep()


if __name__ == "__main__":
    main()
