# pragma once

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <eth_ip_driver/eth_ip_driver.hpp>


using hardware_interface::return_type;

namespace fanuc
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    static constexpr size_t POSITION_INTERFACE_INDEX = 0;
    static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;
    static constexpr size_t NUM_JOINTS = 6;

class JointComms : public rclcpp::Node
{
public:
    JointComms();

    // Standard joint state publisher for RViz visualization
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // Joint command/feedback publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_position_joint;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_position_joint;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_velocity_joint;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_velocity_joint;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_torque_joint;

    // Cartesian pose publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_position_cartesian;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr feedback_position_cartesian;

    // Status/diagnostics
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_state;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr robot_diagnostics;

    // Tool pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_position;

    // Trajectory command
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr command_trajectory;
};

class HARDWARE_INTERFACE_PUBLIC HwFanuc : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  // export interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  const std::vector<std::string> standard_interfaces_ = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};

  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_position_command_;
private:
  rclcpp::Logger logger_ = rclcpp::get_logger("hw_fanuc");
  std::shared_ptr<JointComms> comms_;
  std::vector<std::string> joint_names_; 
  std::vector<double> joint_pos_  ;
  std::shared_ptr<eth_ip_driver> EIP_driver_;
};

typedef HwFanuc GenericRobot;

}