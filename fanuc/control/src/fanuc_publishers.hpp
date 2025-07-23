#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class JointComms;  // Forward declaration

class HwFanucPublishers {
public:
    static void init(rclcpp::Node *node, JointComms *owner);
};