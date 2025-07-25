
#include <ethernet_ip.hpp>
#include <vector>
#include <iostream>
#include <cassert>
#include <math.h>

// Constructor: Initializes Ethernet/IP connection to the robot controller
ethernet_ip::ethernet_ip(std::string ip)
{
  ip_ = ip;
  int port = 0xAF12;  // Default Ethernet/IP port used by Fanuc controller

  try
  {
    // Log the connection attempt
    std::string msg = "Attempting connection with IP address " + ip_ + " : " + std::to_string(port);
    Logger(LogLevel::ERROR) << msg;

    // Create and initialize a new EIP session (shared pointer)
    si_ = std::make_shared<eipScanner::SessionInfo>(ip_, port);
  }
  catch (const std::exception& e)
  {
    // Log error to both std::cerr and the custom logger
    std::cerr << e.what() << " — connection not available. Check IP.\n";
    Logger(LogLevel::ERROR) << "\033[1;31m \n\n\n Connection not available. Check IP \n\n\n\033[0m\n";
    return;
  }

  // Set the logging level for the EIP Scanner library
  Logger::setLogLevel(LogLevel::ERROR);
}

// Destructor: Cleans up the Ethernet/IP connection (if needed in the future)
ethernet_ip::~ethernet_ip() 
{
  // Currently empty – relies on smart pointers for cleanup.
  // You can add logging here if you want to track disconnections.
}

// Reads the current joint positions from the robot
// Returns: a std::vector<double> with 6 joint positions in radians
std::vector<double> ethernet_ip::get_current_joint_pos()
{
  // Send a CIP request to read attribute 1 of instance 1 in class 0x7E (Fanuc specific)
  auto response = messageRouter_->sendRequest(
      si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7E, 0x01, 0x01));

  // Extract raw binary data from the response (each joint value is 4 bytes = float)
  std::vector<uint8_t> data = response.getData();
  int numFloats = data.size() / 4;

  std::vector<float> raw;  // Temporary container for float joint values

  // Convert each 4-byte block into a float using memcpy (avoids aliasing issues)
  for (int j = 0; j < numFloats; ++j) 
  {
    float f;
    std::memcpy(&f, &data[j * 4], sizeof(float));
    raw.push_back(f);
  }

  // Extract joint angles from the response
    // Skip the first value (index 0), which is usually a status or timestamp
    // Extract the next 6 values (indices 1 through 6) for the 6 robot joints
    std::vector<double> j_val(raw.begin() + 1, raw.begin() + 7);

    // Convert joint angles from degrees to radians
    // This is standard practice for consistency with ROS and most robotics frameworks
    for (double& angle : j_val)
        angle *= M_PI / 180.0;

    // Fanuc-specific adjustment:
    // Joint 3 is mechanically coupled with Joint 2 on many Fanuc arms,
    // so we add Joint 2's value to Joint 3 to get the correct kinematic position.
    j_val.at(2) += j_val.at(1);

    // Return the final joint values (in radians) as a 6-element vector
    return j_val;
}

/// Reads the current Cartesian pose of the robot from EIP service.
// Returns a 6-element vector: [X, Y, Z, Rx, Ry, Rz] in millimeters and degrees.
std::vector<double> ethernet_ip::get_current_pose()
{
  // Send GET_ATTRIBUTE_SINGLE request to Class 0x7D (typically Cartesian data)
  auto response = messageRouter_->sendRequest(
      si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7D, 0x01, 0x01));

  // Extract raw byte buffer
  std::vector<uint8_t> data = response.getData();
  int numFloats = data.size() / 4;
  std::vector<float> raw;

   // Convert byte data into float values (each float = 4 bytes)
  for (int j = 0; j < numFloats; ++j)
  {
    float f;
    std::memcpy(&f, &data[j * 4], sizeof(float));
    raw.push_back(f);
  }

  // Extract Cartesian pose values (skip first element, e.g., status or timestamp)
  std::vector<double> cart_val(raw.begin() + 1, raw.begin() + 7);

  // NOTE: Typically, position is in mm and orientation (Rx, Ry, Rz) in degrees
  return cart_val;
}

// Reads an integer value from a specified Fanuc integer register.
// `reg` is the register number (e.g., R[1] => reg = 1).
// Returns the integer value stored in the register.
int ethernet_ip::read_register(const int& reg)
{
  // Send GET_ATTRIBUTE_SINGLE request to Class 0x6B (Integer Registers)
  auto response = messageRouter_->sendRequest(
      si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x6B, 0x01, reg));

  // Extract raw bytes and interpret them as an integer
  std::vector<uint8_t> data = response.getData();
  //auto data = response->getData();  // or response.getData() depending on your wrapper
  int val;
  std::memcpy(&val, &data[0], sizeof(int));

  return val;
}

// Writes an integer value to a specific Fanuc integer register.
//
// Parameters:
// - val: The integer value to write (e.g., 1, 42, etc.)
// - reg: The target register number (e.g., R[1] → reg = 1)
//
// This uses the CIP service SET_ATTRIBUTE_SINGLE to write to
// Class ID 0x6B, which corresponds to Fanuc integer registers.
void ethernet_ip::write_register(const int val, const int reg)
{
  // Prepare a buffer with the integer value formatted as CIP DINT (4 bytes)
  Buffer buffer;
  buffer << CipDint(val);

  // Send the SET_ATTRIBUTE_SINGLE request to write the value
  messageRouter_->sendRequest(
      si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x6B, 0x01, reg), buffer.data());
}

// Writes a 6-element joint position vector to a Fanuc position register.
//
// Parameters:
// - j_vals: Joint values in radians (vector of size 6)
// - reg: Register index (e.g., PR[1] => reg = 1)
//
// This function converts joint values to degrees (Fanuc expects degrees),
// applies compensation for joint 3 (Fanuc-specific kinematics), then
// sends the data to the robot controller using CIP over Ethernet/IP.
void ethernet_ip::write_pos_register(std::vector<double> j_vals, const int reg)
{
  // Convert from radians to degrees
  for (double& val : j_vals)
    val *= 180.0 / M_PI;

  // Fanuc-specific joint 3 compensation: reverse the offset from joint 2
  j_vals.at(2) -= j_vals.at(1);

  Buffer buffer;

   // First value is a dummy placeholder (often used for timestamp or status)
  buffer << CipReal(0.0);

  // Add the 6 joint angles (converted to float) to the buffer
  for (int i = 0; i < 6; ++i)
    buffer << CipReal(static_cast<float>(j_vals[i]));

  // Add 3 trailing floats (zero padding) — Fanuc uses 10 floats for joint pos
  buffer << CipReal(0.0) << CipReal(0.0) << CipReal(0.0);

  // Send the data to position register via CIP SET_ATTRIBUTE_SINGLE
  // Class 0x7C = Position Registers
  messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x7C, 0x01, reg), buffer.data());
}

// Sends a digital input buffer to the robot controller over Ethernet/IP.
//
// Parameters:
// - buffer: Binary data (e.g., for DPM commands or boolean flags)
//
// Returns:
// - true if the message was sent (success assumed; no response check)
//
// Note:
// - Class 0x04: Assembly Object
// - Instance 151: Likely custom Fanuc-defined assembly instance
// - Attribute 0x03: Target attribute for writing data
bool ethernet_ip::write_DI(const Buffer buffer)
{
  messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x04, 151, 0x03), buffer.data());
  return true;
}

// Sends a boolean flag to a specific position over Ethernet/IP.
// Typically used to control tools like grippers or safety signals.
//
// Parameters:
// - val:   The boolean value to send (true = set, false = clear)
// - pos:   Not used in the message but possibly used for addressing in future
//
// Returns:
// - true (always, but logs an error if the write failed)
bool ethernet_ip::write_flag(const bool val, const int pos)
{
  Buffer buffer;
  buffer << CipBool(val); // Encode the boolean into the buffer

  auto response = messageRouter_->sendRequest(
    si_, 
    ServiceCodes::SET_ATTRIBUTE_SINGLE, 
    EPath(0x04, 0x342, 0x03), // Class 0x04 (Assembly), Instance 0x342, Attribute 0x03
    buffer.data());

  // Check if the controller accepted the command
  if (response.getGeneralStatusCode() != GeneralStatusCodes::SUCCESS)
    Logger(LogLevel::ERROR) << __LINE__ << " — Failed to write flag: 0x" << std::hex << response.getGeneralStatusCode();

  return true;
}

// Writes a set of six raw integer values to the robot's DPM (Data Position Monitor) input buffer.
// These values are usually pre-scaled externally (e.g., for position control or other I/O logic).
//
// Parameters:
// - vals: A vector of 6 integers representing the desired DPM values.
//
// Returns:
// - true if the vector size is correct and the write operation is initiated,
//   false if the input vector size is invalid.
bool ethernet_ip::writeDPM(const std::vector<int> vals)
{
  // Validate input vector size
  if (vals.size() != 6) {
    Logger(LogLevel::ERROR) << "Expected vector of size 6! Got: " << vals.size();
    return false;
  }

  // Build buffer for transmission
  Buffer buffer;
  for (const auto& v : vals)
    buffer << CipInt(v); // Write each integer as a CIP 16-bit integer

  buffer << CipInt(0);  // End flag (required by Fanuc DPM protocol)
  
  // Send to DI interface (mapped in controller)
  write_DI(buffer);
  return true;
}

// Converts floating-point values to scaled ints and writes DPM input
bool ethernet_ip::writeDPMScaled(const std::vector<double> vals, const double scale)
{
  if (vals.size() != 6) {
    Logger(Log
