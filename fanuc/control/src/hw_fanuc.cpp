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
#include "hw_fanuc.h"
//#include "fanuc_publishers.hpp"

namespace fanuc
{
    // Constructor for the communication node handling publishers
    JointComms::JointComms() : Node("hw_fanuc")
    {
        //HwFanucPublishers::init(this, this); // Initialize publishers

        /// Publisher for standard joint states (positions, velocities, efforts)
        // Typically used by RViz and other visualization/debugging tools.
       
        
        // Publishers for joint command and feedback:
        // - command_position_joint: publishes desired joint positions to control the robot.
        // - feedback_position_joint: publishes actual joint positions read from the robot.
        // - command_velocity_joint: publishes desired joint velocities for velocity control.
        // - feedback_velocity_joint: publishes measured joint velocities.
        // - feedback_torque_joint: publishes torque/effort feedback per joint.
        command_position_joint = this->create_publisher<sensor_msgs::msg::JointState>("/command_position_joint", 10);

        feedback_position_joint = this->create_publisher<sensor_msgs::msg::JointState>("/feedback_position_joint",10);

        feedback_position_cartesian = this->create_publisher<sensor_msgs::msg::JointState>("/feedback_position_cartesian", 10);
               
        
    }

    // Utility function to safely parse a double from a string
    double parse_double(const std::string& text) 
    {
        double result_value;
        const auto parse_result = std::from_chars(text.data(), text.data() + text.size(), result_value);
        if (parse_result.ec == std::errc())
        {
            return result_value;
        }

        return 0.0;
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

        std::string ro = info_.hardware_parameters["read_only"];
        boost::algorithm::to_lower(ro);
        RCLCPP_INFO_STREAM(logger_,"\n RO::" << ro);
        read_only_ = ( ro =="true") ? true : false;

        if(read_only_)
        RCLCPP_INFO_STREAM(logger_,"\n read only mode active. the robot can be moved from this hardware interface " );

        std::string rmi = info_.hardware_parameters["use_rmi"];
        boost::algorithm::to_lower(rmi);
        RCLCPP_FATAL_STREAM(logger_,"\n using RMI" << rmi);  
        useRMI_ = ( rmi =="true") ? true : false;

        if(useRMI_)
            RCLCPP_INFO_STREAM(logger_,"\n Using RMI to control the robot !   " );
        else
            RCLCPP_INFO_STREAM(logger_,"\n Using Ethernet/IP to control the robot !   " );

        std::string robot_ip = info_.hardware_parameters["robot_ip"];
        RCLCPP_INFO_STREAM( logger_,"\n\n\nIP : "<< robot_ip << "\n\n\n\n "  );
        
        if(useRMI_)
        {
            if (!rmi_driver_.init(robot_ip, 6))
            RCLCPP_ERROR_STREAM(logger_,"RMI non initialized. Robot ip: "<<robot_ip);
            RCLCPP_INFO_STREAM(logger_,"RMI  initialized. Robot ip: "<<robot_ip);
        }
        else
        {  
            EIP_driver_.reset( new fanuc_eth_ip (robot_ip) );
            RCLCPP_INFO_STREAM(logger_,"Initialized EIP driver at ip: " << robot_ip );
        }
  
        std::vector<double> j_pos={0.0,0.0,0.0,0.0,0.0,0.0};
  
        joint_position_        .resize(6, 0.0);
        joint_position_prev_   .resize(6, 0.0);
        joint_velocities_      .resize(6, 0.0);
        joint_position_command_.resize(6, 0.0); 

        joint_names_.resize(joint_position_.size());
        for (size_t j = 0; j < joint_position_.size(); ++j)
        {
            joint_names_.at(j) = info_.joints[j].name;
            RCLCPP_DEBUG_STREAM(logger_,info_.joints[j].name);
        }

        // Read current joint position from the robot via Ethernet/IP
        if(useRMI_)
        {
            j_pos = rmi_driver_.getPosition();

            for(size_t i=0;i<j_pos.size();i++)
            RCLCPP_WARN_STREAM(logger_,j_pos.at(i));
        }
        else
        {
            j_pos = EIP_driver_->get_current_joint_pos();
            EIP_driver_->setCurrentPos();
        }

        for(size_t i=0;i<joint_position_.size();i++)
        {
            joint_position_command_.at(i) = j_pos.at(i);
        }

        if(useRMI_)
        {
            rmi_driver_.setTargetPosition(joint_position_command_);
        }
        else
        {
            EIP_driver_->write_register(1,1);
            // EIP_driver_->write_pos_register(joint_position_command_);
        }
        comms_ = std::make_shared<JointComms>();
        executor_.add_node(comms_);
        std::thread([this]() { executor_.spin(); }).detach();

        return CallbackReturn::SUCCESS;

    }

    // Export state interfaces (position & velocity) for each joint
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

    // Export command interfaces (position) for each joint
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

    // Read robot state: joint position, velocity and Cartesian pose
    return_type HwFanuc::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Read joint and Cartesian positions from the EIP driver
        std::vector<double> jp;
        jp.resize(6);
        std::vector<double> cp;
        cp.resize(6);


        if(useRMI_) 
        {
            jp = rmi_driver_.getPosition();

            rclcpp::Rate rate2(100);
            while (!rmi_driver_.instruction_parsed_ && rclcpp::ok())
            {
            rate2.sleep();
            }
            rmi_driver_.instruction_parsed_=false;
        }
        else
        {
            jp = EIP_driver_->get_current_joint_pos();
            cp = EIP_driver_->get_current_pose();
        }

        double dt = period.seconds();
        for (size_t j = 0; j < joint_position_command_.size(); ++j)
        {
            joint_position_[j] = jp[j];
            joint_velocities_[j] = (jp[j] - joint_position_prev_[j]) / dt;
            joint_position_prev_[j] = jp[j];

            RCLCPP_DEBUG_STREAM(logger_,jp[j]);
        }  
        
        {  
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = comms_->get_clock()->now();
            msg.name = joint_names_;
            msg.position = joint_position_;
            msg.velocity = joint_velocities_;
            comms_->feedback_position_joint->publish(msg);
        }
        {
            auto msg = sensor_msgs::msg::JointState();
            msg.header.stamp = comms_->get_clock()->now();
            std::vector<std::string> n = {"x","y","z","Rx","Ry","Rz"};
            msg.name = n;
            msg.position = cp;
            comms_->feedback_position_cartesian->publish(msg);
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(logger_,"READ time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]" );

        return return_type::OK;
    }

    // Write commanded joint positions to the robot
    return_type HwFanuc::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        auto read_only_ = false;
        if(!read_only_)
        {
            if(useRMI_)
            {
            rmi_driver_.setTargetPosition(joint_position_command_);
            }
            else
            {
            if(EIP_driver_->read_register(RegisterEnum::MotionStatus) == StatusEnum::Ros )
            {
                RCLCPP_DEBUG_STREAM(logger_,"DPM INACTIVE");
                EIP_driver_->write_pos_register(joint_position_command_);
            }
            else
            {
                joint_position_command_ = joint_position_;
                EIP_driver_->write_pos_register(joint_position_command_);
                RCLCPP_DEBUG_STREAM(logger_,"DPM ACTIVE");
            }
            }
        }
        

        

        // Publish the commanded joint positions
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = comms_->get_clock()->now();
        msg.name = joint_names_;
        msg.position = joint_position_command_;
        comms_->command_position_joint->publish(msg);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        RCLCPP_DEBUG_STREAM(logger_,"WRITE time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]" );

        return return_type::OK;
    }
}

// Register the hardware plugin with ROS 2 pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fanuc::HwFanuc, hardware_interface::SystemInterface)