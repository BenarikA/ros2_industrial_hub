// Standard C++ headers
#include <algorithm>
#include <charconv>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <chrono>
#include<unistd.h>

// Boost for string manipulation
#include <boost/algorithm/string.hpp>

// ROS 2 and hardware interface headers
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/rclcpp.hpp"

// Custom Fanuc-specific headers
#include "fanuc/hw_fanuc.h"
#include "hw_fanuc_publishers.hpp"

namespace fanuc
{
    // Constructor for the communication node handling publishers
    JointComms::JointComms() : Node("hw_fanuc")
    {
        HwFanucPublishers::init(this, this); // Initialize publishers
    }

    // Utility function to safely parse a double from a string
    double parse_double(const std::string& text) 
    {
        double result;
        auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), result);
        return (ec == std::errc()) ? result : 0.0;
    }

    // Initialization function for the HwFanuc hardware interface.
    // It sets up communication with the robot and prepares joint state tracking.
    CallbackReturn HwFanuc::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Log that initialization has started
        RCLCPP_INFO(logger_, "init hw_fanuc");

        // Initialize base SystemInterface and check if it was successful
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Set internal flags: not using RMI, not read-only — we want to send commands
        read_only_ = false;
        useRMI_ = false;

        // Fetch and print robot IP from hardware parameters (warn if empty)
        RCLCPP_INFO_STREAM(logger_,"\n Using Ethernet/IP to control the robot.\n");
        if (robot_ip.empty()) 
        {
            RCLCPP_WARN(logger_, "\n\nNo robot_ip provided in hardware parameters!\n\n");
        }
        else
        {
            std::string robot_ip = info_.hardware_parameters["robot_ip"];
            RCLCPP_INFO_STREAM(logger_,"\n\n IP : "<< robot_ip << "\n\n ");
        }

        // Create and initialize the Ethernet/IP driver using the robot IP
        EIP_driver_.reset( new ethernet_ip (robot_ip) );
        RCLCPP_INFO_STREAM(logger_,"\n\nInitialized EIP driver at ip: " << robot_ip );
        
        // Define and initialize joint state vectors with 6 zeros (for a 6-axis robot)
        std::vector<double> j_pos={0.0,0.0,0.0,0.0,0.0,0.0};
        std::vector<double> init_joints(6, 0.0);
  
        joint_position_ = init_joints;
        joint_velocities_ = init_joints;
        joint_position_command_ = init_joints;

        // Prepare joint names from URDF or config
        joint_names_.clear();
        joint_names_.reserve(info_.joints.size());

        for (const auto &joint : info_.joints)
        {
            joint_names_.emplace_back(joint.name);
            RCLCPP_DEBUG_STREAM(logger_, "Joint name: " << joint.name);
        }

        // Read current joint position from the robot via Ethernet/IP
        j_pos = EIP_driver_->get_current_joint_pos();

        // Set the driver's internal current position
        EIP_driver_->setCurrentPos();

        // Initialize the command buffer with the current joint positions
        for(size_t i=0;i<joint_position_.size();i++)
        {
            joint_position_command_.at(i) = j_pos.at(i);
        }

        // Write to a register to notify robot controller (e.g., signal ready)
        EIP_driver_->write_register(1,1);

        // Initialize communication publishers (e.g., for RViz and topic debugging)
        comms_ = std::make_shared<JointComms>();

        // Add the comms node to an executor and run it in a background thread
        executor_.add_node(comms_);
        std::thread([this]() { executor_.spin(); }).detach();

        // Initialization complete
        return CallbackReturn::SUCCESS;
    }

    // Export state interfaces (position & velocity) for each joint
    std::vector<hardware_interface::StateInterface> HwFanuc::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (size_t i = 0; i < 6; ++i)
        {
            state_interfaces.emplace_back(info_.joints[i].name, "position", &joint_position_[i]);
            state_interfaces.emplace_back(info_.joints[i].name, "velocity", &joint_velocities_[i]);
        }      

        return state_interfaces;
    }

    // Export command interfaces (position) for each joint
    std::vector<hardware_interface::CommandInterface> HwFanuc::export_command_interfaces()
    {

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < 6; ++i)
        {
            command_interfaces.emplace_back(info_.joints[i].name, "position", &joint_position_command_[i]);
        }
        
        return command_interfaces;
    }

    // Read robot state: joint position, velocity and Cartesian pose
    return_type HwFanuc::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Read joint and Cartesian positions from the EIP driver
        std::vector<double> jp = EIP_driver_->get_current_joint_pos();
        std::vector<double> cp = EIP_driver_->get_current_pose();

        // Compute joint velocities using finite difference
        // Safety clamp on dt to avoid division by zero
        double dt = period.seconds();
        if (dt < 1e-6) dt = 1e-6;

        // Ensure size consistency before updating states
        if (jp.size() != joint_position_command_.size()) {
            RCLCPP_ERROR(logger_, "Joint position size mismatch!");
            return return_type::ERROR;
        }

        // Update joint state estimates from last known joint positions
        for (size_t j = 0; j < joint_position_command_.size(); ++j)
        {
            joint_position_[j] = jp[j];
            joint_velocities_[j] = (jp[j] - joint_position_prev_[j]) / dt;
            joint_position_prev_[j] = jp[j];

            RCLCPP_DEBUG_STREAM(logger_, "Joint " << j << ": " << jp[j]);
        }

        // Publish joint states for feedback
        {
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = comms_->get_clock()->now();
            msg.name = joint_names_;
            msg.position = joint_position_;
            msg.velocity = joint_velocities_;
            comms_->feedback_position_joint->publish(msg);  // match your naming
        }

        // Publish Cartesian pose as JointState with xyz + rotations or consider PoseStamped
        {
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = comms_->get_clock()->now();
            msg.name = {"x", "y", "z", "Rx", "Ry", "Rz"};
            msg.position = cp;
            comms_->feedback_position_cartesian->publish(msg);  // match your naming
        }

        // Log read duration
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(logger_, "READ time: = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [microseconds]");

        return return_type::OK;
    }

    // Write commanded joint positions to the robot
    return_type HwFanuc::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        auto begin = std::chrono::steady_clock::now();

        if (!read_only_)
        {
            if (EIP_driver_)
            {
            auto status = EIP_driver_->read_register(RegisterEnum::MotionStatus);

            if (status == StatusEnum::Ros)
            {
                RCLCPP_DEBUG(logger_, "DPM INACTIVE");
                EIP_driver_->write_pos_register(joint_position_command_);
            }
            else
            {
                joint_position_command_ = joint_position_; // Fall back to current joint position
                EIP_driver_->write_pos_register(joint_position_command_);
                RCLCPP_DEBUG(logger_, "DPM ACTIVE");
            }
            }
            else
            {
            RCLCPP_ERROR(logger_, "EIP driver is null");
            return return_type::ERROR;
            }
        }
        

        

        // Publish the commanded joint positions
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = comms_->get_clock()->now();
        msg.name = joint_names_;
        msg.position = joint_position_command_;
        msg.velocity = joint_velocities_;
        msg.effort = joint_effort_;
        comms_->command_position_joint->publish(msg);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(logger_,"WRITE time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]" );

        return return_type::OK;
    }
}

// Register the hardware plugin with ROS 2 pluginlib
PLUGINLIB_EXPORT_CLASS(fanuc::HwFanuc, hardware_interface::SystemInterface)