import copy
import math
import threading
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pymoveit2 import MoveIt2


class MoveItGoalNode(Node):
    def __init__(self):
        super().__init__("moveit_goal_node")

        self.declare_parameter("goal_pose_topic", "/mirobot_goal_pose")
        self.declare_parameter("group_name", "mirobot_group")
        self.declare_parameter("base_link_name", "base_link")
        self.declare_parameter("end_effector_name", "link6")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter("cartesian", False)
        self.declare_parameter("cartesian_max_step", 0.0025)
        self.declare_parameter("cartesian_fraction_threshold", 0.0)
        self.declare_parameter("execute", True)
        self.declare_parameter("ignore_same_goal", True)
        self.declare_parameter("dry_run_only", False)
        self.declare_parameter("accept_any_frame", False)

        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.base_link_name = str(self.get_parameter("base_link_name").value)
        self.end_effector_name = str(self.get_parameter("end_effector_name").value)
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.cartesian = bool(self.get_parameter("cartesian").value)
        self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value)
        self.cartesian_fraction_threshold = float(
            self.get_parameter("cartesian_fraction_threshold").value
        )
        self.execute_motion = bool(self.get_parameter("execute").value)
        self.ignore_same_goal = bool(self.get_parameter("ignore_same_goal").value)
        self.dry_run_only = bool(self.get_parameter("dry_run_only").value)
        self.accept_any_frame = bool(self.get_parameter("accept_any_frame").value)

        self.get_logger().info(
            "MoveItGoalNode parameter \n"
            "  group        : %s\n"
            "  base_link    : %s\n"
            "  end_effector : %s\n"
            "  joint_names  : %s\n"
            "  cartesian    : %s\n"
            "  execute      : %s\n"
            "  dry_run_only : %s\n"
            "  accept_frame : %s"
            % (
                self.group_name,
                self.base_link_name,
                self.end_effector_name,
                self.joint_names,
                self.cartesian,
                self.execute_motion,
                self.dry_run_only,
                self.accept_any_frame,
            )
        )
        
        self._last_goal: Optional[PoseStamped] = None  
        self._busy = False
        self._lock = threading.Lock()

        self.moveit2 = None
        if not self.dry_run_only:
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.group_name,
            )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self.goal_pose_callback,
            10,
        )

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(
            "Received MoveIt goal request: frame=%s x=%.4f y=%.4f z=%.4f"
            % (
                msg.header.frame_id,
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            )
