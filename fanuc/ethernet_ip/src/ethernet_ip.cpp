
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
    si_.reset( new eipScanner::SessionInfo( ip_,  port) );
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
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  // Send a CIP request to read attribute 1 of instance 1 in class 0x7E (Fanuc specific)
  auto response = messageRouter_->sendRequest(
      si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7E, 0x01, 0x01));

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  // Extract raw binary data from the response (each joint value is 4 bytes = float)
  std::vector<uint8_t> data = response.getData();
  int numFloats = data.size() / 4;

  std::vector<float> full;  // Temporary container for float joint values

  // Convert each 4-byte block into a float using memcpy (avoids aliasing issues)
  for (int j = 0; j < numFloats; ++j) 
  {
    unsigned char byteArray[4];
    for (int i = 0; i < 4; ++i)
      byteArray[i] = myList[j * 4 + i];

    float floatValue;
    std::memcpy(&floatValue, byteArray, sizeof(float));
    full.push_back(floatValue);
  }

  // Extract joint angles from the response
    // Skip the first value (index 0), which is usually a status or timestamp
    // Extract the next 6 values (indices 1 through 6) for the 6 robot joints
    std::vector<double> j_val(raw.begin() + 1, raw.begin() + 7);

    // Convert joint angles from degrees to radians
    // This is standard practice for consistency with ROS and most robotics frameworks
    for (int i=0;i<j_val.size();i++)
      j_val.at(i) *= M_PI/180.0;

    // Fanuc-specific adjustment:
    // Joint 3 is mechanically coupled with Joint 2 on many Fanuc arms,
    // so we add Joint 2's value to Joint 3 to get the correct kinematic position.
    j_val.at(2) += j_val.[1];

    // Return the final joint values (in radians) as a 6-element vector
    return j_val;
}

/// Reads the current Cartesian pose of the robot from EIP service.
// Returns a 6-element vector: [X, Y, Z, Rx, Ry, Rz] in millimeters and degrees.
std::vector<double> ethernet_ip::get_current_pose()
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  // Send GET_ATTRIBUTE_SINGLE request to Class 0x7D (typically Cartesian data)
  auto response = messageRouter_->sendRequest(
      si_, ServiceCodes::GET_ATTRIBUTE_SINGLE, EPath(0x7D, 0x01, 0x01));

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  // Extract raw byte buffer
  std::vector<uint8_t> data = response.getData();
  int numFloats = data.size() / 4;
  std::vector<float> full;

   // Convert byte data into float values (each float = 4 bytes)
  for (int j = 0; j < numArrays; ++j) 
  {
    unsigned char byteArray[4];
    for (int i = 0; i < 4; ++i)
      byteArray[i] = myList[j * 4 + i];

    float floatValue;
    std::memcpy(&floatValue, byteArray, sizeof(float));
    full.push_back(floatValue);
  }

  // Extract Cartesian pose values (skip first element, e.g., status or timestamp)
  std::vector<double> cart_val(full.begin()+1, full.begin() + 7);

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
  std::vector<uint8_t> myList = response.getData();
  //auto data = response->getData();  // or response.getData() depending on your wrapper
  int numArrays = myList.size() / 4;
  unsigned char byteArray[4];

  for (int i = 0; i < 4; ++i)
    byteArray[i] = myList[i];

  int intValue;
  std::memcpy(&intValue, byteArray, sizeof(int));

  return intValue;
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
  CipDint arg = val;
  buffer << arg;

  // Send the SET_ATTRIBUTE_SINGLE request to write the value
  auto response = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x6B, 0x01, reg), buffer.data());
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
  ffor (int i=0;i<j_vals.size();i++)
    j_vals.at(i) *= 180.0/M_PI;

  // Fanuc-specific joint 3 compensation: reverse the offset from joint 2
  j_vals.at(2) -= ( j_vals[1]);

  Buffer buffer;

   // First value is a dummy placeholder (often used for timestamp or status)
  buffer  << CipReal(  0.0  )
          << CipReal( static_cast<float>( j_vals[0] ) )
          << CipReal( static_cast<float>( j_vals[1] ) )
          << CipReal( static_cast<float>( j_vals[2] ) )
          << CipReal( static_cast<float>( j_vals[3] ) )
          << CipReal( static_cast<float>( j_vals[4] ) )
          << CipReal( static_cast<float>( j_vals[5] ) )
          << CipReal(  0.0  )
          << CipReal(  0.0  )
          << CipReal(  0.0  ) ;

  // Add the 6 joint angles (converted to float) to the buffer
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  // Add 3 trailing floats (zero padding) — Fanuc uses 10 floats for joint pos

  // Send the data to position register via CIP SET_ATTRIBUTE_SINGLE
  // Class 0x7C = Position Registers
  auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x7C, 0x01, reg), buffer.data());

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
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
  auto response2 = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x04, 151, 0x03), buffer.data());
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
  buffer  << CipBool( true );// Encode the boolean into the buffer

  auto response = messageRouter_->sendRequest(si_, ServiceCodes::SET_ATTRIBUTE_SINGLE, EPath(0x04, 0x342, 0x03), buffer.data());

  // Check if the controller accepted the command
  if (response.getGeneralStatusCode() == GeneralStatusCodes::SUCCESS) {
    Buffer buffer(response.getData());
    CipUint vendorId;
    buffer >> vendorId;

    Logger(LogLevel::ERROR) << "size  " << buffer.size();
    Logger(LogLevel::ERROR) << "response " << vendorId;
  } else {
    Logger(LogLevel::ERROR) << __LINE__ <<"We____ got error=0x" << std::hex << response.getGeneralStatusCode();
  }

  std::vector<uint8_t> data = response.getData();
  for (auto d:data)
    Logger(LogLevel::ERROR) << __LINE__ <<"data" << d ;


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
  for(auto v:vals)
    buffer  << CipInt( v );
  buffer  << CipInt( 0 );
  write_DI(buffer);

  return true;
}

// Converts floating-point values to scaled ints and writes DPM input
bool ethernet_ip::writeDPMScaled(const std::vector<double> vals, const double scale)
{
  if(vals.size()!=6)
  {
    Logger(LogLevel::ERROR) << "expected vector of size 6! got: " << vals.size();
    return false;
  }

  std::vector<int> v(6);
  for (int i=0;i<vals.size();i++)
    v.at(i)=vals.at(i)/scale;

  writeDPM(v);

  return true;
}
// Activate DPM
void ethernet_ip::activateDPM(const bool activate)
{
  setCurrentPos();
  
  if(activate)
    write_register(StatusEnum::Dpm, RegisterEnum::MotionStatus );
  else
    write_register(StatusEnum::Ros, RegisterEnum::MotionStatus );

  int act = (activate) ? 0 : 1;
  Buffer buffer;
  for(int i=0;i<6;i++)
    buffer  << CipInt( 0 );
  buffer  << CipInt( act );
  write_DI(buffer);
  write_register(act, RegisterEnum::DpmStatus);
  
    Logger(LogLevel::ERROR) << "\033[1;31m \n\n\n DPM activate: "<< act <<" \n\n\n\033[0m\n";

}
// Deactivate DPM
void ethernet_ip::deactivateDPM(){activateDPM(false);}

void ethernet_ip::setCurrentPos() {write_pos_register(get_current_joint_pos());}

