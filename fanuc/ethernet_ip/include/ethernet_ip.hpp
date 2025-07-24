#pragma once

// Required headers from the EIPScanner library for Ethernet/IP communication
#include <EIPScanner/MessageRouter.h>
#include <EIPScanner/utils/Logger.h>
#include <EIPScanner/utils/Buffer.h>

// Shorten names from the EIPScanner library for easier usage
using eipScanner::SessionInfo;
using eipScanner::MessageRouter;
using namespace eipScanner::cip;
using namespace eipScanner::utils;

// Fanuc-specific register mapping for Ethernet/IP communication
enum RegisterEnum 
{
    MotionStatus    = 1,   // Robot motion status
    DpmStatus       = 9,   // DPM (Direct Position Manipulation) status
    GripperMove     = 10,  // Gripper movement command
    GripperPos      = 11,  // Gripper position
    GripperVel      = 12,  // Gripper velocity
    GripperEff      = 13,  // Gripper effort/force
    GripperActivate = 15,  // Gripper activation toggle
    CollabActive    = 99   // Collaborative mode status
};

// Internal status tracking
enum StatusEnum 
{
    Inactive = 0, // Robot not active
    Ros      = 1, // ROS control mode
    Dpm      = 2  // DPM control mode
};


// Main interface class for communicating with a Fanuc robot over Ethernet/IP
class ethernet_ip
{
private:
    std::string ip_;      // IP address of the robot controller
    std::shared_ptr<SessionInfo> si_;  // Session info object for EIP communication
    std::shared_ptr<MessageRouter> messageRouter_ = std::make_shared<MessageRouter>();

public:
    // Constructor and destructor
    ethernet_ip(std::string ip);
    ~ethernet_ip();

    // Reads joint angles from the robot (typically 6 for industrial arms)
    std::vector<double> get_current_joint_pos();

    // Reads the robot's TCP pose (x, y, z, rx, ry, rz)
    std::vector<double> get_current_pose();

    // Read a robot register (used for state/mode information)
    int read_register(const int& reg);

    // Write a value to a robot register
    void write_register(const int val, const int reg = 1);

    // Write joint values to a position register
    void write_pos_register(const std::vector<double> j_vals, const int reg = 1);

    // Write a digital input buffer to the robot
    bool write_DI(const Buffer buffer);

    // Write a boolean flag to a digital output
    bool write_flag(const bool val, const int pos);

    // Enable DPM mode (Direct Position Manipulation)
    void activateDPM(const bool activate = true);

    // Disable DPM mode
    void deactivateDPM();

    // Write raw joint values to DPM input
    bool writeDPM(const std::vector<int> vals);

    // Write scaled joint values to DPM input (e.g. radians or degrees -> internal units)
    bool writeDPMScaled(const std::vector<double> vals, const double scale = 0.01);

    // Set current joint position as reference
    void setCurrentPos();
};