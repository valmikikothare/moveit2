#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>
#include <moveit_py/moveit_py_utils/copy_ros_msg.hpp>
#include <moveit_py/moveit_py_utils/ros_msg_typecasters.hpp>
#include <moveit/trajectory_cache/trajectory_cache.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>

namespace py = pybind11;

namespace moveit_py
{
namespace bind_trajectory_cache
{
// Forward declaration if needed, or potentially include trajectory_cache.hpp
// If we define everything in the cpp file, this header might just need includes
// for pybind11

// Function to define the bindings (optional to declare here)
void initTrajectoryCache(py::module_& m);

}  // namespace bind_trajectory_cache
}  // namespace moveit_py