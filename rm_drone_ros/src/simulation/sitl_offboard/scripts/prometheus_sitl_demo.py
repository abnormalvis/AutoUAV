#!/usr/bin/env python3
"""Run one Prometheus basic demo after an explicit SITL-only GO trigger."""

import os
import subprocess
import time

import rospkg
import rospy
from mavros_msgs.msg import RCIn
from nav_msgs.msg import Odometry
from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVState
from std_msgs.msg import Empty, String


DEMO_LAUNCHES = {
    "takeoff_land": "takeoff_land/launch/takeoff_land.launch",
    "enu_xyz_pos_control": "enu_xyz_pos_control/launch/enu_xyz_pos_control.launch",
    "body_xyz_pos_control": "body_xyz_pos_control/launch/body_xyz_pos_control.launch",
    "circular_trajectory_control": "circular_trajectory_control/launch/circular_trajectory_control.launch",
}


class PrometheusSitlDemo:
    def __init__(self):
        self.uav_id = rospy.get_param("~uav_id", 1)
        self.demo = rospy.get_param("~demo", "enu_xyz_pos_control")
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.0)
        self.takeoff_tolerance = rospy.get_param("~takeoff_tolerance", 0.12)
        self.state_timeout = rospy.get_param("~state_timeout", 2.0)
        self.takeoff_timeout = rospy.get_param("~takeoff_timeout", 30.0)
        self.go_received = False
        self.state = UAVState()
        self.control_state = UAVControlState()
        self.have_state = False
        self.have_control_state = False
        self.rc_channels = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.sequence = 0

        if self.demo not in DEMO_LAUNCHES:
            raise ValueError("Unsupported demo: " + self.demo)
        if self.uav_id != 1:
            raise ValueError("This first SITL integration supports uav_id=1 only")

        prefix = "/uav{}/prometheus".format(self.uav_id)
        self.state_sub = rospy.Subscriber(prefix + "/state", UAVState, self._state_cb, queue_size=1)
        self.control_state_sub = rospy.Subscriber(
            prefix + "/control_state", UAVControlState, self._control_state_cb, queue_size=1
        )
        self.go_sub = rospy.Subscriber("/prometheus/sitl_demo/go", Empty, self._go_cb, queue_size=1)
        self.status_pub = rospy.Publisher("/prometheus/sitl_demo/status", String, queue_size=1, latch=True)
        self.rc_pub = rospy.Publisher(prefix + "/fake_rc_in", RCIn, queue_size=1)
        self.command_pub = rospy.Publisher(prefix + "/command", UAVCommand, queue_size=1)
        self.rate = rospy.Rate(20)

    def _state_cb(self, message):
        self.state = message
        self.have_state = True

    def _control_state_cb(self, message):
        self.control_state = message
        self.have_control_state = True

    def _go_cb(self, _message):
        self.go_received = True

    def _status(self, value):
        rospy.loginfo("[prometheus_sitl_demo] %s", value)
        self.status_pub.publish(String(data=value))

    def _publish_rc(self):
        message = RCIn()
        message.header.stamp = rospy.Time.now()
        message.channels = list(self.rc_channels)
        self.rc_pub.publish(message)

    def _wait_until(self, predicate, description, timeout):
        start = rospy.get_time()
        while not rospy.is_shutdown():
            self._publish_rc()
            if predicate():
                return True
            if rospy.get_time() - start > timeout:
                raise RuntimeError("Timed out waiting for " + description)
            self.rate.sleep()
        return False

    def _prepare_flight(self):
        self._status("WAITING_FOR_GO")
        while not rospy.is_shutdown() and not self.go_received:
            self.rate.sleep()
        if rospy.is_shutdown():
            return False

        self._status("CHECKING_FCU_AND_ODOMETRY")
        if not self._wait_until(
            lambda: self.have_state and self.have_control_state and self.state.connected and self.state.odom_valid,
            "connected FCU and valid Gazebo odometry",
            self.state_timeout * 10,
        ):
            return False
        if self.state.armed:
            raise RuntimeError("Refusing to start: SITL vehicle is already armed")

        self._status("ARMING_SITL")
        for _ in range(10):
            self._publish_rc()
            self.rate.sleep()
        self.rc_channels[4] = 2000
        if not self._wait_until(lambda: self.state.armed, "SITL arm", self.state_timeout * 10):
            return False

        self._status("ENTERING_HOVER_MODE")
        self.rc_channels[5] = 1500
        if not self._wait_until(
            lambda: self.control_state.control_state == UAVControlState.RC_POS_CONTROL and self.state.mode == "OFFBOARD",
            "Prometheus RC_POS_CONTROL and PX4 OFFBOARD",
            self.state_timeout * 15,
        ):
            return False

        self._status("ENTERING_COMMAND_CONTROL")
        self.rc_channels[5] = 2000
        if not self._wait_until(
            lambda: self.control_state.control_state == UAVControlState.COMMAND_CONTROL,
            "Prometheus COMMAND_CONTROL",
            self.state_timeout * 10,
        ):
            return False

        command = UAVCommand()
        command.header.stamp = rospy.Time.now()
        command.header.frame_id = "ENU"
        command.Agent_CMD = UAVCommand.Init_Pos_Hover
        command.Command_ID = self.sequence = 1
        self.command_pub.publish(command)
        self._status("TAKING_OFF_TO_{:.2f}M".format(self.takeoff_height))
        reached_height = self._wait_until(
            lambda: self.state.position[2] >= self.takeoff_height - self.takeoff_tolerance,
            "takeoff height",
            self.takeoff_timeout,
        )
        if reached_height:
            self._status("READY_FOR_" + self.demo.upper())
        return reached_height

    def _launch_demo(self):
        package_path = rospkg.RosPack().get_path("prometheus_demo")
        launch_file = os.path.join(package_path, "basic", DEMO_LAUNCHES[self.demo])
        self._status("RUNNING_" + self.demo.upper())
        process = subprocess.Popen(["roslaunch", launch_file])
        while not rospy.is_shutdown() and process.poll() is None:
            self.rate.sleep()
        return process.wait()

    def run(self):
        try:
            if not self._prepare_flight():
                return
            result = self._launch_demo()
            self._status("DEMO_EXITED_{}".format(result))
            if result != 0:
                raise RuntimeError("Prometheus demo exited with status {}".format(result))
            self._status("WAITING_FOR_LANDING")
            if self._wait_until(lambda: not self.state.armed, "SITL disarm after landing", 60.0):
                self._status("COMPLETE")
        except Exception as error:
            self._status("ERROR: " + str(error))
            rospy.logerr("Prometheus SITL demo stopped: %s", error)


if __name__ == "__main__":
    rospy.init_node("prometheus_sitl_demo")
    PrometheusSitlDemo().run()