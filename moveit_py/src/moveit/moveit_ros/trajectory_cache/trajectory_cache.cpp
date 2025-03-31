#include "trajectory_cache.hpp"

namespace moveit_py
{
namespace bind_trajectory_cache
{

void initTrajectoryCache(py::module_& m)
{
  py::class_<TrajectoryCache, std::shared_ptr<TrajectoryCache>>(m, "TrajectoryCache")
      .def(py::init<const rclcpp::Node::SharedPtr&>(), py::arg("node"), "Constructor")
      // Bind Options struct if necessary, or handle options directly in init
      .def("init", &TrajectoryCache::init, py::arg("options"), "Initialize the trajectory cache.")
      .def("count_trajectories", &TrajectoryCache::countTrajectories, py::arg("cache_namespace"),
           "Count the number of motion plan trajectories in the specified namespace.")
      .def("count_cartesian_trajectories", &TrajectoryCache::countCartesianTrajectories, py::arg("cache_namespace"),
           "Count the number of cartesian trajectories in the specified namespace.")
      .def("get_db_path", &TrajectoryCache::getDbPath, "Get the database path.")
      .def("get_db_port", &TrajectoryCache::getDbPort, "Get the database port.")
      .def("get_exact_match_precision", &TrajectoryCache::getExactMatchPrecision, "Get the exact match precision.")
      .def("set_exact_match_precision", &TrajectoryCache::setExactMatchPrecision, py::arg("exact_match_precision"),
           "Set the exact match precision.")
      .def("get_num_additional_trajectories_to_preserve_when_deleting_worse",
           &TrajectoryCache::getNumAdditionalTrajectoriesToPreserveWhenDeletingWorse,
           "Get the number of additional trajectories to preserve when deleting worse ones.")
      .def("set_num_additional_trajectories_to_preserve_when_deleting_worse",
           &TrajectoryCache::setNumAdditionalTrajectoriesToPreserveWhenDeletingWorse,
           py::arg("num_additional_trajectories_to_preserve_when_deleting_worse"),
           "Set the number of additional trajectories to preserve when deleting worse ones.")

      // Motion Plan Caching Methods
      .def("fetch_all_matching_trajectories", &TrajectoryCache::fetchAllMatchingTrajectories, py::arg("move_group"),
           py::arg("cache_namespace"), py::arg("plan_request"), py::arg("start_tolerance"), py::arg("goal_tolerance"),
           py::arg("metadata_only"), py::arg("sort_by") = "execution_time_s", py::arg("ascending") = true,
           "Fetch all trajectories matching the request.")
      .def("fetch_best_matching_trajectory", &TrajectoryCache::fetchBestMatchingTrajectory, py::arg("move_group"),
           py::arg("cache_namespace"), py::arg("plan_request"), py::arg("start_tolerance"), py::arg("goal_tolerance"),
           py::arg("metadata_only"), py::arg("sort_by") = "execution_time_s", py::arg("ascending") = true,
           "Fetch the best trajectory matching the request.")
      .def("insert_trajectory", &TrajectoryCache::insertTrajectory, py::arg("move_group"), py::arg("cache_namespace"),
           py::arg("plan_request"), py::arg("trajectory"), py::arg("execution_time_s"), py::arg("planning_time_s"),
           py::arg("prune_worse_trajectories") = true, "Insert a motion plan trajectory into the cache.")

      // Cartesian Trajectory Caching Methods
      .def("construct_get_cartesian_path_request", &TrajectoryCache::constructGetCartesianPathRequest,
           py::arg("move_group"), py::arg("waypoints"), py::arg("max_step"), py::arg("jump_threshold"),
           py::arg("avoid_collisions"), "Construct a GetCartesianPath request.")
      .def("fetch_all_matching_cartesian_trajectories", &TrajectoryCache::fetchAllMatchingCartesianTrajectories,
           py::arg("move_group"), py::arg("cache_namespace"), py::arg("plan_request"), py::arg("min_fraction"),
           py::arg("start_tolerance"), py::arg("goal_tolerance"), py::arg("metadata_only"),
           py::arg("sort_by") = "fraction", py::arg("ascending") = false,  // Default sort by fraction descending
           "Fetch all cartesian trajectories matching the request.")
      .def("fetch_best_matching_cartesian_trajectory", &TrajectoryCache::fetchBestMatchingCartesianTrajectory,
           py::arg("move_group"), py::arg("cache_namespace"), py::arg("plan_request"), py::arg("min_fraction"),
           py::arg("start_tolerance"), py::arg("goal_tolerance"), py::arg("metadata_only"),
           py::arg("sort_by") = "fraction", py::arg("ascending") = false,  // Default sort by fraction descending
           "Fetch the best cartesian trajectory matching the request.")
      .def("insert_cartesian_trajectory", &TrajectoryCache::insertCartesianTrajectory, py::arg("move_group"),
           py::arg("cache_namespace"), py::arg("plan_request"), py::arg("trajectory"), py::arg("execution_time_s"),
           py::arg("planning_time_s"), py::arg("fraction"), py::arg("prune_worse_trajectories") = true,
           "Insert a cartesian trajectory into the cache.");

  // Bind the Options struct
  py::class_<TrajectoryCache::Options>(m, "TrajectoryCacheOptions")
      .def(py::init<>())
      .def_readwrite("db_path", &TrajectoryCache::Options::db_path)
      .def_readwrite("db_port", &TrajectoryCache::Options::db_port)
      .def_readwrite("exact_match_precision", &TrajectoryCache::Options::exact_match_precision)
      .def_readwrite("num_additional_trajectories_to_preserve_when_deleting_worse",
                     &TrajectoryCache::Options::num_additional_trajectories_to_preserve_when_deleting_worse);

  // Add bindings for helper functions if needed, e.g., sortConstraints
  // m.def("sort_constraints", &sortConstraints, "Sort constraint vectors by name.");
  // Note: Binding free functions might require adjustments based on their definitions and usage.
}

// Define the module entry point
PYBIND11_MODULE(trajectory_cache_py, m)
{
  m.doc() = "Python bindings for the MoveIt Trajectory Cache";  // Optional module docstring

  define_trajectory_cache_bindings(m);

  // Bind other necessary types like moveit_msgs types if they aren't already bound elsewhere
  // Example: py::class_<moveit_msgs::msg::MotionPlanRequest>(m, "MotionPlanRequest")...
  // Example: py::class_<moveit_msgs::msg::RobotTrajectory>(m, "RobotTrajectory")...
  // Example: py::class_<moveit_msgs::srv::GetCartesianPath::Request>(m, "GetCartesianPathRequest")...
  // Example: py::class_<moveit::planning_interface::MoveGroupInterface,
  // std::shared_ptr<moveit::planning_interface::MoveGroupInterface>>(m, "MoveGroupInterface")... Ensure
  // rclcpp::Node::SharedPtr is usable or wrapped if necessary
}

}  // namespace bind_trajectory_cache
}  // namespace moveit_py
