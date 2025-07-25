# pragma once

#include <string>
#include <vector>


// Standard and ROS2 includes for node handling, messaging, and hardware interfaces
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>

#include "sensor_msgs/msg/joint_state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <ethernet_ip.hpp>


using hardware_interface::return_type;

namespace fanuc
{
  // Convenience alias for lifecycle callback return type
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Constants for joint interface indices and robot DOF count
  static constexpr size_t POSITION_INTERFACE_INDEX = 0;
  static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;
  static constexpr size_t NUM_JOINTS = 6;


  // ROS node class managing all relevant publishers for robot communication and visualization
  class JointComms : public rclcpp::Node
  {
  public:
    JointComms();

    // Separate publishers for joint command and feedback data (position, velocity, torque)
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_position_joint;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_position_joint;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_position_cartesian;
};

// Main hardware interface class implementing ROS2 control SystemInterface for the Fanuc robot
class HARDWARE_INTERFACE_PUBLIC HwFanuc : public hardware_interface::SystemInterface
{
public:
  // Lifecycle initialization: parse hardware info, set up communication, etc.
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Export the robot's state interfaces (position, velocity) to the controller manager
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Export command interfaces (desired positions) for external controllers
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Read robot's current joint states from hardware, publish feedback
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Write commanded joint positions to robot hardware
  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  // List of standard hardware interface types supported by this driver
  const std::vector<std::string> standard_interfaces_ = {
  hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
  hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};

  // Internal storage for joint states and commands
  std::vector<double> joint_position_; // Current joint positions (measured)
  std::vector<double> joint_position_prev_; // Current joint positions (measured)
  std::vector<double> joint_velocities_; // Current joint velocities (measured)
  std::vector<double> joint_position_command_; // Commanded joint positions to send
private:
  rclcpp::Logger logger_ = rclcpp::get_logger("hw_fanuc"); // Logger instance for debug/info/error output
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<JointComms> comms_;                       // Node handling all ROS publishers
  std::vector<std::string> joint_names_;                    // Names of robot joints, from URDF/config
  std::vector<double> joint_pos_;                           // Temporary storage for joint positions
  std::shared_ptr<ethernet_ip> EIP_driver_;               // Ethernet/IP driver interface to robot hardware

  rmi::RMIDriver rmi_driver_;

  bool useRMI_;

  bool read_only_;

  template <typename HandleType>
  bool get_interface(
    const std::string & name, const std::vector<std::string> & interface_list,
    const std::string & interface_name, const size_t vector_index,
    std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

  void initialize_storage_vectors(
    std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
    const std::vector<std::string> & interfaces,
    const std::vector<hardware_interface::ComponentInfo> & component_infos);

  template <typename InterfaceType>
  bool populate_interfaces(
    const std::vector<hardware_interface::ComponentInfo> & components,
    std::vector<std::string> & interfaces, std::vector<std::vector<double>> & storage,
    std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces);
};

// Alias for convenience if needed
typedef HwFanuc GenericRobot;

}