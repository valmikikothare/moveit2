from __future__ import annotations
from typing import Iterator, List
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState


class RobotTrajectory:
    """
    Maintains a sequence of waypoints and the durations between these waypoints.
    """

    def __init__(self, robot_model: RobotModel) -> None:
        """
        Initializes an empty robot trajectory from a robot model.
        """
        ...

    def __getitem__(self, idx: int) -> RobotState:
        """
        Get the waypoint at the specified index in the trajectory.
        """
        ...

    def __iter__(self) -> Iterator[RobotState]:
        """
        Iterate over the waypoints in the trajectory.
        """
        ...

    def __len__(self) -> int:
        """
        Returns the number of waypoints in the trajectory.
        """
        ...

    def __lt__(self, other: RobotTrajectory) -> bool:
        """
        Compare two trajectories.
        """
        ...

    @property
    def joint_model_group_name(self) -> str:
        """
        The name of the joint model group that this trajectory is for.
        """
        ...

    @property
    def robot_model(self) -> RobotModel:
        """
        The robot model that this trajectory is for.
        """
        ...

    @property
    def duration(self) -> float:
        """
        The duration of the trajectory.
        """
        ...

    @property
    def average_segment_duration(self) -> float:
        """
        The average duration of the segments in the trajectory.
        """
        ...

    @property
    def path_length(self) -> float:
        """
        The length of the path.
        """
        ...

    def unwind(self) -> None:
        """
        Unwind the trajectory.
        """
        ...

    def reverse(self) -> RobotTrajectory:
        """
        Reverse the trajectory.
        """
        ...

    def append(self, source: RobotTrajectory, dt: float, start_index: int = 0, end_index: int = -1) -> None:
        """
        Append a trajectory to the end of the current trajectory.
        """
        ...

    def get_waypoint_durations(self) -> List[float]:
        """
        Get the durations from the previous waypoint in the trajectory.
        """
        ...

    def apply_totg_time_parameterization(
        self,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        path_tolerance: float = 0.1,
        resample_dt: float = 0.1,
        min_angle_change: float = 0.001,
    ) -> bool:
        """
        Adds time parameterization to the trajectory using the Time-Optimal Trajectory Generation (TOTG) algorithm.
        """
        ...

    def apply_ruckig_smoothing(
        self,
        velocity_scaling_factor: float,
        acceleration_scaling_factor: float,
        mitigate_overshoot: bool = False,
        overshoot_threshold: float = 0.01,
    ) -> bool:
        """
        Applies Ruckig smoothing to the trajectory.
        """
        ...

    def get_robot_trajectory_msg(self, joint_filter: List[str] = []) -> RobotTrajectoryMsg:
        """
        Get the trajectory as a moveit_msgs.msg.RobotTrajectory message.
        """
        ...

    def set_robot_trajectory_msg(self, robot_state: RobotState, msg: RobotTrajectoryMsg) -> RobotTrajectory:
        """
        Set the trajectory from a moveit_msgs.msg.RobotTrajectory message.
        """
        ...
