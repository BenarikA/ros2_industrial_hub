#pragma once

// ROS 2 core and message headers for publishing robot state and diagnostics
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"           // Joint state messages (positions, velocities, efforts)
#include "geometry_msgs/msg/pose_stamped.hpp"        // Pose messages with timestamp (for Cartesian poses)
#include "std_msgs/msg/string.hpp"                    // Standard string message type (e.g., robot state)
#include "diagnostic_msgs/msg/diagnostic_array.hpp"  // Diagnostic info array message type
#include "trajectory_msgs/msg/joint_trajectory.hpp" // Joint trajectory messages for motion commands

class JointComms;  // Forward declaration of JointComms class (defined elsewhere)

/**
 * @brief Helper class responsible for initializing ROS publishers used for Fanuc robot communication.
 * 
 * This class provides a static method to set up the various ROS publishers on the given node,
 * which are then owned/managed by the JointComms node instance.
 * 
 * It abstracts away the publisher creation details to keep JointComms class cleaner and focused.
 */

class HwFanucPublishers {
public:
    /**
     * @brief Initialize all necessary ROS publishers on the provided node.
     * 
     * @param node Pointer to the ROS2 node on which publishers will be created.
     * @param owner Pointer to the JointComms instance which will hold the created publishers.
     */
    static void init(rclcpp::Node *node, JointComms *owner);
};