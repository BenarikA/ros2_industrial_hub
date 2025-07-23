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

#include <boost/algorithm/string.hpp>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/rclcpp.hpp"

#include "fanuc/hw_fanuc.h"
#include "hw_fanuc_publishers.hpp"

namespace fanuc
{
    JointComms::JointComms() : Node("hw_fanuc")
    {
        HwFanucPublishers::init(this, this);
    }

    double parse_double(const std::string& text) 
    {
        double result;
        auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), result);
        return (ec == std::errc()) ? result : 0.0;
    }

    std::vector<hardware_interface::StateInterface> HwFanuc::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(info_.joints[0].name, "position", &joint_position_[0]);
        state_interfaces.emplace_back(info_.joints[1].name, "position", &joint_position_[1]);
        state_interfaces.emplace_back(info_.joints[2].name, "position", &joint_position_[2]);
        state_interfaces.emplace_back(info_.joints[3].name, "position", &joint_position_[3]);
        state_interfaces.emplace_back(info_.joints[4].name, "position", &joint_position_[4]);
        state_interfaces.emplace_back(info_.joints[5].name, "position", &joint_position_[5]);

        state_interfaces.emplace_back(info_.joints[0].name, "velocity", &joint_velocities_[0]);
        state_interfaces.emplace_back(info_.joints[1].name, "velocity", &joint_velocities_[1]);
        state_interfaces.emplace_back(info_.joints[2].name, "velocity", &joint_velocities_[2]);
        state_interfaces.emplace_back(info_.joints[3].name, "velocity", &joint_velocities_[3]);
        state_interfaces.emplace_back(info_.joints[4].name, "velocity", &joint_velocities_[4]);
        state_interfaces.emplace_back(info_.joints[5].name, "velocity", &joint_velocities_[5]);

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> HwFanuc::export_command_interfaces()
    {

        std::vector<hardware_interface::CommandInterface> command_interfaces;


        command_interfaces.emplace_back(info_.joints[0].name, "position", &joint_position_command_[0]);
        command_interfaces.emplace_back(info_.joints[1].name, "position", &joint_position_command_[1]);
        command_interfaces.emplace_back(info_.joints[2].name, "position", &joint_position_command_[2]);
        command_interfaces.emplace_back(info_.joints[3].name, "position", &joint_position_command_[3]);
        command_interfaces.emplace_back(info_.joints[4].name, "position", &joint_position_command_[4]);
        command_interfaces.emplace_back(info_.joints[5].name, "position", &joint_position_command_[5]);

        return command_interfaces;
    }

    return_type HwFanuc::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        std::vector<double> jp = EIP_driver_->get_current_joint_pos();
        std::vector<double> cp = EIP_driver_->get_current_pose();

        double dt = period.seconds();
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

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(logger_, "READ time: = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " [microseconds]");

        return return_type::OK;
    }

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
                joint_position_command_ = joint_position_;
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

PLUGINLIB_EXPORT_CLASS(fanuc::HwFanuc, hardware_interface::SystemInterface)