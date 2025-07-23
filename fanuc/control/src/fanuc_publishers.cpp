#include "fanuc/fanuc_publishers.hpp"
#include "fanuc/HwFanuc.h"  // Includes definition for JointComms

// fanuc_publishers.cpp
void HwFanucPublishers::init(rclcpp::Node *node, JointComms *owner)
{
    // Publishers for joint-level commands and feedback:
    // - command_position_joint: publishes desired joint positions (setpoints) to control the robot.
    // - feedback_position_joint: publishes the actual measured joint positions from the robot.
    // - command_velocity_joint: publishes desired joint velocities for velocity control mode.
    // - feedback_velocity_joint: publishes measured joint velocities from the robot.
    // - feedback_torque_joint: publishes torque or effort feedback for each joint.

    owner->joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    owner->command_position_joint = node->create_publisher<sensor_msgs::msg::JointState>("/command_position_joint", 10);
    owner->feedback_position_joint = node->create_publisher<sensor_msgs::msg::JointState>("/feedback_position_joint",10);
    owner->command_velocity_joint = node->create_publisher<sensor_msgs::msg::JointState>("/command_velocity_joint", 10);
    owner->feedback_velocity_joint = node->create_publisher<sensor_msgs::msg::JointState>("/feedback_velocity_joint", 10);
    owner->feedback_torque_joint = node->create_publisher<sensor_msgs::msg::JointState>("/feedback_torque_joint", 10);
    
    // Publisher for Cartesian feedback:
    // - command_position_cartesian: publishes desired end-effector pose (position + orientation) commands in Cartesian space.
    // - feedback_position_cartesian: publishes actual measured end-effector pose from the robot for feedback and monitoring.

    owner->command_position_cartesian = node->create_publisher<geometry_msgs::msg::PoseStamped>("/command_position_cartesian", 10);
    owner->feedback_position_cartesian = node->create_publisher<geometry_msgs::msg::PoseStamped>("/feedback_position_cartesian",10);

    // Publishers for overall robot status and diagnostics:
    // - robot_state: publishes the current state of the robot (e.g., "idle", "moving", "error").
    // - robot_diagnostics: publishes detailed diagnostic information for monitoring and troubleshooting.

    owner->robot_state = node->create_publisher<std_msgs::msg::String>("/robot_state", 10);
    owner->robot_diagnostics = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/robot_diagnostics", 10);

    // Publisher for the tool/end-effector pose:
    // - tool_position: publishes the current pose (position + orientation) of the tool in 3D space.

    owner->tool_position = node->create_publisher<geometry_msgs::msg::PoseStamped>("/tool_position", 10);

    // Publisher for joint trajectory commands:
    // - command_trajectory: publishes full joint trajectory messages to command smooth robot motion paths.

    owner->command_trajectory = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/command_trajectory", 10);
}