cmake_minimum_required(VERSION 3.8)
project(ethernet_ip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(rcpputils REQUIRED)

include_directories(/usr/local/include/EIPScanner)
include_directories(include)


set(HW_IF_INCLUDE_DEPENDS
  rcpputils
  rclcpp
)

add_library(ethernet_ip SHARED  src/ethernet_ip.cpp)

target_link_libraries(ethernet_ip EIPScanner)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ethernet_ip
  EXPORT ethernet_ip
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
ament_export_libraries(ethernet_ip)


ament_export_targets(ethernet_ip HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


ament_package()
