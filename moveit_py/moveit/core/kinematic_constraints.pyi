from __future__ import annotations
from typing import Optional, List
from moveit_msgs.msg import Constraints
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import JointModelGroup
from rclpy.node import Node


def construct_link_constraint(
    link_name: str,
    source_frame: str,
    cartesian_position: Optional[List[float]] = None,
    cartesian_position_tolerance: Optional[float] = None,
    orientation: Optional[List[float]] = None,
    orientation_tolerance: Optional[float] = None,
) -> Constraints:
    """
    Construct a link constraint message.
    """
    ...


def construct_joint_constraint(
    robot_state: RobotState,
    joint_model_group: JointModelGroup,
    tolerance: float = 0.01,
) -> Constraints:
    """
    Construct a joint constraint message.
    """
    ...


def construct_constraints_from_node(node: Node, ns: str) -> Constraints:
    """
    Construct a constraint message from a node.
    """
    ...
