#!/usr/bin/env python3
"""
Conveyor controller: laser trigger -> stop belt -> pick and place -> resume belt.

  belt running --(beam broken)--> belt stopped --(settle)--> /conveyor/cube_ready = True
      ^                                                              |
      |                                                              v
      +---- belt resumes <----(/pick_place/done)---- pick_place_node runs the sequence

Topics (ROS side, bridged to Gazebo by conveyor_control.launch.py):
  in   /conveyor/laser_scan   sensor_msgs/LaserScan   beam across the belt end
  out  /conveyor/cmd_vel      std_msgs/Float64        roller speed [rad/s], 0 = stopped
  out  /conveyor/cube_ready   std_msgs/Bool           True = cube in position, pick it now
  in   /pick_place/done       std_msgs/Bool           pick_place_node finished (True = success)

The belt-state logic lives in BeltLogic (pure Python, no ROS) so it can be tested alone.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

RUNNING = "RUNNING"          # belt on, watching the beam
SETTLING = "SETTLING"        # belt off, waiting for the cube to settle
WAITING = "WAITING_PICK"     # trigger sent, waiting for pick_place_node


class BeltLogic:
    """Belt state machine. Methods return a list of events: 'stopped', 'ready', 'resumed', 'timeout'."""

    def __init__(self, run_speed, threshold, debounce, settle_time, done_timeout,
                 rearm_delay, startup_delay, now):
        self.run_speed = run_speed
        self.threshold = threshold          # range below this = beam broken [m]
        self.debounce = debounce            # consecutive broken scans needed
        self.settle_time = settle_time      # belt stopped -> trigger [s]
        self.done_timeout = done_timeout    # give up waiting for the pick [s]
        self.rearm_delay = rearm_delay      # ignore the beam after resuming [s]
        self.state = RUNNING
        self.speed = run_speed
        self.broken = 0
        self.state_since = now
        self.ignore_until = now + startup_delay

    def on_scan(self, ranges, range_min, now):
        if self.state != RUNNING or now < self.ignore_until:
            self.broken = 0
            return []
        hit = any(math.isfinite(r) and range_min <= r < self.threshold for r in ranges)
        self.broken = self.broken + 1 if hit else 0
        if self.broken >= self.debounce:
            self.broken = 0
            self.speed = 0.0
            self.state = SETTLING
            self.state_since = now
            return ["stopped"]
        return []

    def tick(self, now):
        elapsed = now - self.state_since
        if self.state == SETTLING and elapsed >= self.settle_time:
            self.state = WAITING
            self.state_since = now
            return ["ready"]
        if self.state == WAITING and elapsed >= self.done_timeout:
            return ["timeout"] + self._resume(now)
        return []

    def on_done(self, now):
        if self.state != WAITING:
            return []
        return self._resume(now)

    def _resume(self, now):
        self.speed = self.run_speed
        self.state = RUNNING
        self.state_since = now
        self.ignore_until = now + self.rearm_delay
        self.broken = 0
        return ["resumed"]


class ConveyorController(Node):
    def __init__(self):
        super().__init__("conveyor_controller_node")
        self.declare_parameter("scan_topic", "/conveyor/laser_scan")
        self.declare_parameter("cmd_topic", "/conveyor/cmd_vel")
        self.declare_parameter("ready_topic", "/conveyor/cube_ready")
        self.declare_parameter("done_topic", "/pick_place/done")
        self.declare_parameter("run_speed", 12.0)        # rad/s; + moves the cubes toward the arm
        self.declare_parameter("beam_threshold", 0.35)   # m; the cube face is ~0.22 m from the laser
        self.declare_parameter("debounce_scans", 5)      # ~0.17 s at 30 Hz
        self.declare_parameter("settle_time", 1.0)       # s
        self.declare_parameter("done_timeout", 180.0)    # s
        self.declare_parameter("rearm_delay", 3.0)       # s
        self.declare_parameter("startup_delay", 2.0)     # s

        p = self.get_parameter
        self.logic = BeltLogic(
            run_speed=p("run_speed").value,
            threshold=p("beam_threshold").value,
            debounce=p("debounce_scans").value,
            settle_time=p("settle_time").value,
            done_timeout=p("done_timeout").value,
            rearm_delay=p("rearm_delay").value,
            startup_delay=p("startup_delay").value,
            now=self.now(),
        )

        # The trigger is latched so a pick_place_node that starts late still sees it, and it is
        # reset to False when the cycle ends so a restarted node does not replay an old trigger.
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Float64, p("cmd_topic").value, 10)
        self.ready_pub = self.create_publisher(Bool, p("ready_topic").value, latched)
        self.create_subscription(LaserScan, p("scan_topic").value, self.on_scan, 10)
        self.create_subscription(Bool, p("done_topic").value, self.on_done, 10)
        self.create_timer(0.2, self.on_tick)

        self.ready_pub.publish(Bool(data=False))
        self.publish_speed()
        self.get_logger().info(
            f"Belt running at {self.logic.speed} rad/s, waiting for the laser beam to break")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_speed(self):
        self.cmd_pub.publish(Float64(data=float(self.logic.speed)))

    def handle(self, events):
        for ev in events:
            if ev == "stopped":
                self.get_logger().info("Laser beam broken -> cube at the end of the belt, stopping belt")
            elif ev == "ready":
                self.get_logger().info("Belt settled -> triggering pick and place")
                self.ready_pub.publish(Bool(data=True))
            elif ev == "timeout":
                self.get_logger().warn("Pick and place did not report back in time, resuming belt anyway")
            elif ev == "resumed":
                self.ready_pub.publish(Bool(data=False))
                self.get_logger().info("Resuming belt")
        if events:
            self.publish_speed()

    def on_scan(self, msg: LaserScan):
        self.handle(self.logic.on_scan(msg.ranges, msg.range_min, self.now()))

    def on_done(self, msg: Bool):
        if self.logic.state == WAITING:
            self.get_logger().info(
                "Pick and place finished" + ("" if msg.data else " (reported failure)"))
        self.handle(self.logic.on_done(self.now()))

    def on_tick(self):
        self.handle(self.logic.tick(self.now()))
        self.publish_speed()      # re-sent at 5 Hz so a dropped first message cannot matter


def main():
    rclpy.init()
    node = ConveyorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
