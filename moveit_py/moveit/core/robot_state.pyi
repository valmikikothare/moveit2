from __future__ import annotations

from typing import Optional

import numpy
from geometry_msgs.msg import Pose
from moveit.core.robot_model import JointModelGroup, RobotModel
from moveit_msgs.msg import RobotState as RobotStateMsg

def robotStateToRobotStateMsg(
    state: RobotState, copy_attached_bodies: bool = True
) -> RobotStateMsg: ...

class RobotState:
    """
    Representation of a robot's state. At the lowest level, a state is a collection of variables. Each variable has a name and can have position, velocity, acceleration and effort associated to it. Effort and acceleration share the memory area for efficiency reasons (one should not set both acceleration and effort in the same state and expect things to work). Often variables correspond to joint names as well (joints with one degree of freedom have one variable), but joints with multiple degrees of freedom have more variables. Operations are allowed at variable level, joint level (see JointModel) and joint group level (see JointModelGroup). For efficiency reasons a state computes forward kinematics in a lazy fashion. This can sometimes lead to problems if the update() function was not called on the state.
    """

    def __init__(self, robot_model: RobotModel) -> None: ...
    def __copy__(self) -> RobotState: ...
    def __deepcopy__(self) -> RobotState: ...
    @property
    def robot_model(self) -> RobotModel:
        """
        The robot model instance associated to this robot state.
        """
        ...
    @property
    def dirty(self) -> bool:
        """
        True if the robot state is dirty.
        """
        ...
    def get_frame_transform(self, frame_id: str) -> numpy.ndarray:
        """
        Get the transformation matrix from the model frame (root of model) to the frame identified by frame_id. If frame_id was not found, frame_found is set to false and an identity transform is returned. This method is restricted to frames defined within the robot state and doesn't include collision object present in the collision world. Please use the PlanningScene.getFrameTransform method for collision world objects.
        """
        ...
    def get_pose(self, link_name: str) -> Pose:
        """
        Get the pose of a link that is defined in the robot model.
        """
        ...
    def get_jacobian(
        self,
        joint_model_group_name: str,
        reference_point_position: numpy.ndarray,
        link_name: Optional[str] = None,
        use_quaternion_representation: bool = False,
    ) -> numpy.ndarray:
        """
        Compute the Jacobian with reference to a particular point on a given link, for a specified group.
        """
        ...
    @property
    def state_tree(self) -> str:
        """
        Represents the state tree of the robot state.
        """
        ...
    @property
    def state_info(self) -> str:
        """
        The state information of the robot state.
        """
        ...
    @property
    def joint_positions(self) -> dict[str, float]: ...
    @joint_positions.setter
    def joint_positions(self, joint_positions: dict[str, float]) -> None: ...
    @property
    def joint_velocities(self) -> dict[str, float]: ...
    @property
    def joint_accelerations(self) -> dict[str, float]: ...
    @property
    def joint_efforts(self) -> dict[str, float]: ...
    def set_joint_group_positions(
        self, joint_model_group_name: str, position_values: numpy.ndarray
    ) -> None:
        """
        Sets the positions of the joints in the specified joint model group.
        """
        ...
    def set_joint_group_active_positions(
        self, joint_model_group_name: str, position_values: numpy.ndarray
    ) -> None:
        """
        Sets the active positions of joints in the specified joint model group.
        """
        ...
    def get_joint_group_positions(
        self, joint_model_group_name: str
    ) -> numpy.ndarray:
        """
        For a given group, get the position values of the variables that make up the group.
        """
        ...
    def set_joint_group_velocities(
        self, joint_model_group_name: str, velocity_values: numpy.ndarray
    ) -> None:
        """
        Sets the velocities of the joints in the specified joint model group.
        """
        ...
    def get_joint_group_velocities(
        self, joint_model_group_name: str
    ) -> numpy.ndarray:
        """
        For a given group, get the velocity values of the variables that make up the group.
        """
        ...
    def set_joint_group_accelerations(
        self, joint_model_group_name: str, acceleration_values: numpy.ndarray
    ) -> None:
        """
        Sets the accelerations of the joints in the specified joint model group.
        """
        ...
    def get_joint_group_accelerations(
        self, joint_model_group_name: str
    ) -> numpy.ndarray:
        """
        For a given group, get the acceleration values of the variables that make up the group.
        """
        ...
    def get_global_link_transform(self, link_name: str) -> numpy.ndarray:
        """
        Returns the transform of the specified link in the global frame.
        """
        ...
    def set_from_ik(
        self,
        joint_model_group_name: str,
        geometry_pose: Pose,
        tip_name: str,
        timeout: float = 0.0,
    ) -> bool:
        """
        Sets the state of the robot to the one that results from solving the inverse kinematics for the specified group.
        """
        ...
    def set_to_default_values(
        self,
        joint_model_group: Optional[JointModelGroup] = None,
        name: Optional[str] = None,
    ) -> None:
        """
        Set all joints to their default positions.
        The default position is 0, or if that is not within bounds then half way between min and max bound.
        """
        ...
    def set_to_random_positions(
        self, joint_model_group: Optional[JointModelGroup] = None
    ) -> None:
        """
        Set all joints to random positions within the default bounds.
        """
        ...
    def clear_attached_bodies(self) -> None:
        """
        Clear all attached bodies. We only allow for attaching of objects via the PlanningScene instance. This method allows any attached objects that are associated to this RobotState instance to be removed.
        """
        ...
    def update(self, force: bool = False, category: str = "all") -> None:
        """
        Update state transforms.
        """
        ...
