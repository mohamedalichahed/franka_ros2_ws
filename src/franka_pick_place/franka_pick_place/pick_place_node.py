#!/usr/bin/env python3
"""
Simple fixed-pose pick-and-place for the Franka FR3 using moveit_py.

This is a *fixed-pose* demo: the pick and place locations are hardcoded
constants below. Perception (detecting the object's actual pose) is meant
to replace PICK_POSE later -- see the TODO near the bottom.

Sequence: home -> approach pick -> descend -> close gripper -> retreat
          -> approach place -> descend -> open gripper -> retreat -> home
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState

from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
# ---------------------------------------------------------------------------
# TUNABLE CONSTANTS -- adjust these to match your actual setup.
# ---------------------------------------------------------------------------

PLANNING_GROUP = "fr3_arm"
PLANNING_PIPELINE = "ompl"

# Gripper action name. Verify with: ros2 action list | grep gripper
GRIPPER_ACTION_NAME = "/fr3_gripper/gripper_cmd"

# World-frame poses (robot base "fr3_link0" assumed at world origin).
# Both are within the FR3's 855mm nominal reach -- see conversation notes
# for how these were derived from the conveyor's spawn position.
PICK_POSE = {"x": 0.50, "y": 0.0, "z": 0.495}   # z = belt top surface (object bottom)
PLACE_POSE = {"x": 0.35, "y": 0.55, "z": 0.41}  # z = table top (0.40) + 1 cm drop

# Downward orientation: gripper pointing straight down (roll=pi, pitch=0, yaw=0
# as a quaternion). This is the common "top-down grasp" orientation.
DOWNWARD_ORIENTATION = {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}

APPROACH_HEIGHT_OFFSET = 0.15   # hover this far above the pick/place pose first
OBJECT_HALF_HEIGHT = 0.04   # TCP height above the cube bottom (5 cm cube)
GRASP_WIDTH_OPEN = 0.04         # gripper opening when open (meters)
GRASP_WIDTH_CLOSED = 0.0        # per-finger target: fully closed, so the fingers squeeze
GRIPPER_MAX_EFFORT = 40.0
GRASP_SETTLE_TIME = 1         # seconds to hold still after closing, before lifting
GRIPPER_RESULT_TIMEOUT = 3.0    # max seconds to wait for a gripper result before moving on

# "Local" moves (descend / lift): the goal joint state comes from IK seeded at the CURRENT
# joints, so the arm takes the nearby solution instead of spinning to a different one.
LOCAL_MOVE_VELOCITY_SCALE = 0.3   # slow, gentle vertical moves
MAX_LOCAL_JOINT_JUMP = 1.0        # rad; refuse a "local" goal further than this from the current joints
IK_TIMEOUT = 0.2                  # seconds for the seeded IK solve
TCP_LINK = "fr3_hand_tcp"

PLANNING_TIME = 5.0
NUM_PLANNING_ATTEMPTS = 10


def make_pose(x, y, z, orientation=DOWNWARD_ORIENTATION):
    pose = PoseStamped()
    pose.header.frame_id = "fr3_link0"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = orientation["x"]
    pose.pose.orientation.y = orientation["y"]
    pose.pose.orientation.z = orientation["z"]
    pose.pose.orientation.w = orientation["w"]
    return pose


def make_box(object_id, size, x, y, z):
    """A box collision object in the fr3_link0 frame (arm base = world origin)."""
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [float(v) for v in size]
    co = CollisionObject()
    co.id = object_id
    co.header.frame_id = "fr3_link0"
    co.operation = CollisionObject.ADD
    co.primitives.append(box)
    pose = PoseStamped().pose
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    co.primitive_poses.append(pose)
    return co


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_place_node")

        # --- MoveIt setup -----------------------------------------------
        self.moveit = MoveItPy(node_name="moveit_py_pick_place")
        self.arm = self.moveit.get_planning_component(PLANNING_GROUP)

        # --- Gripper action client ----------------------------------------
        self.gripper_client = ActionClient(self, GripperCommand, GRIPPER_ACTION_NAME)
        self.get_logger().info(f"Waiting for gripper action server '{GRIPPER_ACTION_NAME}'...")
        self.gripper_available = self.gripper_client.wait_for_server(timeout_sec=10.0)
        if not self.gripper_available:
            self.get_logger().error(
                f"Gripper action server '{GRIPPER_ACTION_NAME}' not available. "
                "Check the name with: ros2 action list | grep gripper. "
                "Gripper commands will be skipped rather than hang."
            )
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cube_ready = False
        self.create_subscription(Bool, '/conveyor/cube_ready', self.on_cube_ready, latched)
        self.done_pub = self.create_publisher(Bool, '/pick_place/done', 10)
    # -----------------------------------------------------------------
    def add_scene_objects(self):
        """Tell MoveIt about the belt and table so paths avoid them. The cylinder is
        deliberately left out: the gripper has to reach it."""
        objects = [
            # belt: x 0.42..2.02, y +-0.2, z 0.40..0.495 (legs ignored)
            make_box("belt", (1.6, 0.4, 0.095), 1.22, 0.0, 0.4475),
            # table: 0.4 x 0.4, solid block up to the top surface at z = 0.40
            make_box("table", (0.4, 0.4, 0.40), 0.35, 0.55, 0.20),
        ]
        psm = self.moveit.get_planning_scene_monitor()
        with psm.read_write() as scene:
            for co in objects:
                scene.apply_collision_object(co)
            scene.current_state.update()
        self.get_logger().info("Added belt and table to the planning scene")

    # -----------------------------------------------------------------
    def _local_goal_state(self, pose: PoseStamped):
        """IK for `pose`, seeded with the current joint values so the solution is the
        nearby one (no elbow flip / joint-1 spin). Returns (RobotState, None) on success,
        (None, "unavailable") if the Python API is missing something, or
        (None, "<reason>") if the goal must be refused."""
        try:
            psm = self.moveit.get_planning_scene_monitor()
            with psm.read_only() as scene:
                current = np.array(
                    scene.current_state.get_joint_group_positions(PLANNING_GROUP), dtype=float)
            goal_state = RobotState(self.moveit.get_robot_model())
            goal_state.set_joint_group_positions(PLANNING_GROUP, current)
            goal_state.update()
            found = goal_state.set_from_ik(PLANNING_GROUP, pose.pose, TCP_LINK, IK_TIMEOUT)
            goal_state.update()
            solved = np.array(
                goal_state.get_joint_group_positions(PLANNING_GROUP), dtype=float)
        except Exception as exc:  # API differences between moveit_py versions
            self.get_logger().warn(f"Seeded IK not available ({exc!r})")
            return None, "unavailable"
        if not found:
            return None, "IK found no solution near the current joints"
        jump = float(np.max(np.abs(solved - current)))
        self.get_logger().info(f"Seeded IK ok, largest joint change = {jump:.3f} rad")
        if jump > MAX_LOCAL_JOINT_JUMP:
            return None, (f"IK solution is {jump:.2f} rad away from the current joints "
                          f"(limit {MAX_LOCAL_JOINT_JUMP}) -- refusing a swinging move")
        return goal_state, None

    def move_to_pose(self, pose: PoseStamped, description: str, local: bool = False) -> bool:
        self.get_logger().info(f"Planning: {description}")
        self.arm.set_start_state_to_current_state()

        goal_state = None
        if local:
            goal_state, problem = self._local_goal_state(pose)
            if goal_state is None and problem != "unavailable":
                self.get_logger().error(f"Cannot plan '{description}': {problem}")
                return False
        if goal_state is not None:
            self.arm.set_goal_state(robot_state=goal_state)
        else:
            if local:
                self.get_logger().warn("Falling back to a plain pose goal (may swing).")
            self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=TCP_LINK)

        scale = LOCAL_MOVE_VELOCITY_SCALE if local else 1.0
        plan_params = PlanRequestParameters(self.moveit)
        plan_params.planning_pipeline = PLANNING_PIPELINE
        plan_params.planning_attempts = NUM_PLANNING_ATTEMPTS
        plan_params.planning_time = PLANNING_TIME
        plan_params.max_velocity_scaling_factor = scale
        plan_params.max_acceleration_scaling_factor = scale

        plan_result = self.arm.plan(plan_params)
        if not plan_result:
            self.get_logger().error(f"Planning failed for: {description}")
            return False

        self.get_logger().info(f"Executing: {description}")
        self.moveit.execute(PLANNING_GROUP, plan_result.trajectory)
        return True

    # -----------------------------------------------------------------
    def command_gripper(self, width: float, description: str) -> bool:
        if not self.gripper_available:
            self.get_logger().warn(
                f"Skipping gripper command ({description}): no action server available."
            )
            return True

        self.get_logger().info(f"Gripper: {description} (width={width})")
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Gripper goal rejected or timed out: {description}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=GRIPPER_RESULT_TIMEOUT)
        if result_future.done():
            res = result_future.result().result
            self.get_logger().info(
                f"Gripper '{description}' done: position={res.position:.4f}, "
                f"reached_goal={res.reached_goal}, stalled={res.stalled}")
        else:
            # Do NOT cancel: the controller keeps squeezing while the goal is active,
            # which is exactly what holds the object during the lift.
            self.get_logger().warn(
                f"Gripper '{description}' gave no result within {GRIPPER_RESULT_TIMEOUT}s "
                "(probably still squeezing the object). Continuing without cancelling.")
        return True

    
    def close_and_settle(self):
        ok = self.command_gripper(GRASP_WIDTH_CLOSED, "close")
        self.get_logger().info(f"Holding grasp for {GRASP_SETTLE_TIME}s before lifting")
        time.sleep(GRASP_SETTLE_TIME)
        return ok
    # -----------------------------------------------------------------
    def run_pick_and_place(self):
        self.add_scene_objects()
        pick = PICK_POSE
        place = PLACE_POSE

        steps = [
            ("open gripper before approach",
             lambda: self.command_gripper(GRASP_WIDTH_OPEN, "open")),

            ("move above pick location",
             lambda: self.move_to_pose(
                 make_pose(pick["x"], pick["y"], pick["z"] + APPROACH_HEIGHT_OFFSET),
                 "approach above pick pose")),

            ("descend to pick object",
             lambda: self.move_to_pose(
                 make_pose(pick["x"], pick["y"], pick["z"] + OBJECT_HALF_HEIGHT),
                 "descend to grasp height", local=True)),

            ("close gripper on object",
             lambda: self.close_and_settle()),

            ("retreat with object",
             lambda: self.move_to_pose(
                 make_pose(pick["x"], pick["y"], pick["z"] + APPROACH_HEIGHT_OFFSET),
                 "retreat above pick pose", local=True)),

            ("move above place location",
             lambda: self.move_to_pose(
                 make_pose(place["x"], place["y"], place["z"] + APPROACH_HEIGHT_OFFSET),
                 "approach above place pose")),

            ("descend to place object",
             lambda: self.move_to_pose(
                 make_pose(place["x"], place["y"], place["z"] + OBJECT_HALF_HEIGHT),
                 "descend to place height", local=True)),

            ("open gripper to release",
             lambda: self.command_gripper(GRASP_WIDTH_OPEN, "release")),

            ("retreat after placing",
             lambda: self.move_to_pose(
                 make_pose(place["x"], place["y"], place["z"] + APPROACH_HEIGHT_OFFSET),
                 "retreat above place pose", local=True)),
        ]

        for description, action in steps:
            self.get_logger().info(f"--- Step: {description} ---")
            success = action()
            if success is False:
                self.get_logger().error(f"Aborting sequence at step: {description}")
                return
            time.sleep(0.5)

        self.get_logger().info("Pick-and-place sequence complete.")

    def on_cube_ready(self, msg):
        if msg.data:
            self.cube_ready = True

    def run_forever(self):
        self.get_logger().info('Waiting for the conveyor laser trigger...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.cube_ready:
                continue
            self.cube_ready = False
            self.run_pick_and_place()
            self.done_pub.publish(Bool(data=True))   # belt resumes even if the pick aborted


def main():
    rclpy.init()
    node = PickPlaceNode()
    try:
        node.run_forever()
    finally:
        del node.moveit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# ---------------------------------------------------------------------------
# TODO (perception integration, later):
#   Replace PICK_POSE with a subscription to a perception topic that
#   publishes the object's actual pose (e.g. from a camera + pose
#   estimation node). Everything else in this node (the move/gripper
#   sequence) stays the same -- only where PICK_POSE comes from changes.
# ---------------------------------------------------------------------------