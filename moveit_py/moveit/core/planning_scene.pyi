from __future__ import annotations
from typing import Optional, List
import numpy
from moveit_msgs.msg import PlanningScene as PlanningSceneMsg
from moveit_msgs.msg import PlanningSceneWorld
from moveit_msgs.msg import CollisionObject as CollisionObjectMsg
from moveit_msgs.msg import AttachedCollisionObject as AttachedCollisionObjectMsg
from moveit_msgs.msg import ObjectColor as ObjectColorMsg
from octomap_msgs.msg import Octomap as OctomapMsg
from moveit_msgs.msg import Constraints
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit.core.robot_trajectory import RobotTrajectory
from moveit.core.collision_detection import (
    CollisionRequest,
    CollisionResult,
    AllowedCollisionMatrix,
    World,
)


class PlanningScene:
    """
    Representation of the environment as seen by a planning instance. The environment geometry, the robot geometry and state are maintained.
    """

    def __init__(self, robot_model: RobotModel, world: Optional[World] = None) -> None: ...
    def __copy__(self) -> PlanningScene: ...
    def __deepcopy__(self) -> PlanningScene: ...
    @property
    def name(self) -> str:
        """
        The name of the planning scene.
        """
        ...
    @property
    def robot_model(self) -> RobotModel:
        """
        The robot model associated to this planning scene.
        """
        ...
    @property
    def planning_frame(self) -> str:
        """
        The frame in which planning is performed.
        """
        ...
    @property
    def current_state(self) -> RobotState:
        """
        The current state of the robot.
        """
        ...
    @property
    def planning_scene_message(self) -> PlanningSceneMsg: ...
    @property
    def transforms(self) -> numpy.ndarray: ...
    @property
    def allowed_collision_matrix(self) -> AllowedCollisionMatrix: ...
    def set_planning_scene_diff_msg(self, scene_msg: PlanningSceneMsg) -> bool:
        """
        Set the planning scene diff message.
        """
        ...
    def set_planning_scene_msg(self, scene_msg: PlanningSceneMsg) -> bool:
        """
        Set the planning scene message.
        """
        ...
    def knows_frame_transform(self, frame_id: str, robot_state: Optional[RobotState] = None) -> bool:
        """
        Check if a transform to the frame id is known. This will be known if id is a link name, an attached body id or a collision object.
        """
        ...
    def get_frame_transform(self, frame_id: str) -> numpy.ndarray:
        """
        Get the transform corresponding to the frame id.
        This will be known if id is a link name, an attached body id or a collision object. Return identity when no transform is available.
        """
        ...
    def process_planning_scene_world(self, msg: PlanningSceneWorld) -> None:
        """
        Process a planning scene world message.
        """
        ...
    def apply_collision_object(
        self,
        collision_object_msg: CollisionObjectMsg,
        color_msg: Optional[ObjectColorMsg] = None,
    ) -> None:
        """
        Apply a collision object to the planning scene.
        """
        ...
    def set_object_color(self, object_id: str, color_msg: ObjectColorMsg) -> None:
        """
        Set the color of a collision object.
        """
        ...
    def process_attached_collision_object(self, object: AttachedCollisionObjectMsg) -> None:
        """
        Apply an attached collision object to the planning scene.
        """
        ...
    def process_octomap(self, msg: OctomapMsg) -> None:
        """
        Apply an octomap to the planning scene.
        """
        ...
    def remove_all_collision_objects(self) -> None:
        """
        Removes collision objects from the planning scene.
        This method will remove all collision object from the scene except for attached collision objects.
        """
        ...
    def set_current_state(self, robot_state: RobotState) -> None:
        """
        Set the current state using a moveit::core::RobotState object.
        """
        ...
    def is_state_valid(
        self,
        robot_state: RobotState,
        joint_model_group_name: str,
        verbose: bool = False,
    ) -> bool: ...
    def is_state_colliding(
        self,
        joint_model_group_name: str,
        verbose: bool = False,
        robot_state: Optional[RobotState] = None,
    ) -> bool:
        """
        Check if the robot state is in collision.
        """
        ...
    def is_state_constrained(
        self,
        state: RobotState,
        constraints: Constraints,
        verbose: bool = False,
    ) -> bool:
        """
        Check if the robot state fulfills the passed constraints
        """
        ...
    def is_path_valid(
        self,
        trajectory: RobotTrajectory,
        joint_model_group_name: str,
        verbose: bool = False,
        invalid_index: Optional[List[int]] = None,
    ) -> bool:
        """
        Check if a given path is valid. Each state is checked for validity (collision avoidance and feasibility)
        """
        ...
    def allocate_collision_detector(self, collision_detector: str) -> None:
        """
        Set the collision detector.
        """
        ...
    def check_collision(
        self,
        collision_request: CollisionRequest,
        collision_result: CollisionResult,
        state: Optional[RobotState] = None,
        acm: Optional[AllowedCollisionMatrix] = None,
    ) -> bool:
        """
        Check if the robot state is in collision.
        """
        ...
    def check_collision_unpadded(
        self,
        collision_request: CollisionRequest,
        collision_result: CollisionResult,
        state: Optional[RobotState] = None,
        acm: Optional[AllowedCollisionMatrix] = None,
    ) -> bool:
        """
        Check if the robot state is in collision.
        """
        ...
    def check_self_collision(
        self,
        collision_request: CollisionRequest,
        collision_result: CollisionResult,
        state: Optional[RobotState] = None,
        acm: Optional[AllowedCollisionMatrix] = None,
    ) -> bool:
        """
        Check if the robot state is in collision.
        """
        ...
    def save_geometry_to_file(self, file_path_and_name: str) -> bool:
        """
        Save the CollisionObjects in the PlanningScene to a file
        """
        ...
    def load_geometry_from_file(self, file_path_and_name: str) -> bool:
        """
        Load the CollisionObjects from a file to the PlanningScene
        """
        ...
