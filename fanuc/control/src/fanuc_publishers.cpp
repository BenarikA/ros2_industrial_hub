#include "fanuc/fanuc_publishers.hpp"
#include "fanuc/HwFanuc.h"  // Includes definition for JointComms

/**
 * @brief Initialize all ROS publishers used for communicating with the Fanuc robot.
 * 
 * This function sets up various publishers on the given ROS node and assigns them to the
 * corresponding publisher members of the JointComms instance (`owner`).
 * These publishers cover joint-level commands and feedback, Cartesian pose data,
 * robot state and diagnostics, tool pose, and joint trajectory commands.
 * 
 * @param node Pointer to the ROS node where publishers will be created.
 * @param owner Pointer to the JointComms instance that will own/manage the publishers.
 */
void HwFanucPublishers::init(rclcpp::Node *node, JointComms *owner)
{
    /// Publisher for standard joint states (positions, velocities, efforts)
    // Typically used by RViz and other visualization/debugging tools.
    owner->joint_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    // Publishers for joint command and feedback:
    // - command_position_joint: publishes desired joint positions to control the robot.
    // - feedback_position_joint: publishes actual joint positions read from the robot.
    // - command_velocity_joint: publishes desired joint velocities for velocity control.
    // - feedback_velocity_joint: publishes measured joint velocities.
    // - feedback_torque_joint: publishes torque/effort feedback per joint.
    owner->command_position_joint = node->create_publisher<sensor_msgs::msg::JointState>("/command_position_joint", 10);
    owner->feedback_position_joint = node->create_publisher<sensor_msgs::msg::JointState>("/feedback_position_joint",10);
    owner->command_velocity_joint = node->create_publisher<sensor_msgs::msg::JointState>("/command_velocity_joint", 10);
    owner->feedback_velocity_joint = node->create_publisher<sensor_msgs::msg::JointState>("/feedback_velocity_joint", 10);
    owner->feedback_torque_joint = node->create_publisher<sensor_msgs::msg::JointState>("/feedback_torque_joint", 10);
    
    // Publishers for Cartesian space commands and feedback:
    // - command_position_cartesian: desired end-effector pose commands (position + orientation).
    // - feedback_position_cartesian: actual end-effector pose from the robot for monitoring.
    owner->command_position_cartesian = node->create_publisher<geometry_msgs::msg::PoseStamped>("/command_position_cartesian", 10);
    owner->feedback_position_cartesian = node->create_publisher<geometry_msgs::msg::PoseStamped>("/feedback_position_cartesian",10);

    // Publishers for robot-wide state and diagnostics information:
    // - robot_state: publishes simple string state info (e.g., "idle", "moving", "error").
    // - robot_diagnostics: publishes detailed diagnostic messages for health monitoring.
    owner->robot_state = node->create_publisher<std_msgs::msg::String>("/robot_state", 10);
    owner->robot_diagnostics = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/robot_diagnostics", 10);

    // Publisher for tool/end-effector pose:
    // Useful to track current position and orientation of the tool in 3D space.
    owner->tool_position = node->create_publisher<geometry_msgs::msg::PoseStamped>("/tool_position", 10);

    // Publisher for joint trajectory commands:
    // Allows commanding smooth, multi-point joint trajectories for the robot.
    owner->command_trajectory = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/command_trajectory", 10);
}