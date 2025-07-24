# Core ROS 2 LaunchDescription class used to define what to launch
from launch import LaunchDescription

# Actions used in launch files:
# - DeclareLaunchArgument: defines arguments that can be passed to this launch file
# - OpaqueFunction: allows for dynamic Python code execution inside the launch file
from launch.actions import DeclareLaunchArgument, OpaqueFunction

# Substitutions are used to dynamically resolve values at launch time:
# - Command: runs a command (e.g., xacro)
# - FindExecutable: locates an executable in the system path (e.g., xacro)
# - LaunchConfiguration: fetches values of declared launch arguments
# - PathJoinSubstitution: safely constructs filesystem paths
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

# Launches a ROS 2 node
from launch_ros.actions import Node

# Finds the share directory of a ROS 2 package (used with PathJoinSubstitution)
from launch_ros.substitutions import FindPackageShare

# Standard Python module for OS path operations (used for joining paths, checking existence, etc.)
import os

# ROS 2 Python API to get the package share directory (used in older or lower-level functions)
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments that can be passed from the command line or other launch files
    declared_arguments = []

    # robot_type will define which robot description (xacro) file to load
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="",
            description="Model of the Fanuc robot.",
        )
    )

    # Return the complete launch description, including the argument declarations
    # and an OpaqueFunction that allows dynamic Python code execution
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
    # Retrieve the value of 'robot_type' from the launch context
    robot_type = LaunchConfiguration("robot_type")
    s_robot_type = robot_type.perform(context)
    print("robot_type", s_robot_type)

    # Name of the package where the robot description (URDF/XACRO) files are stored
    description_package = "fanuc_description"
    
    # Generate the full robot_description by processing the xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf/"+s_robot_type+"/", f"{s_robot_type}.xacro"]),
        ]
    )


    # Set up the parameter dictionary used by robot_state_publisher
    robot_description = {"robot_description": robot_description_content}    

    # Path to the RViz configuration file
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), "config", "config.rviz"])

    # Joint state publisher GUI allows manual adjustment of joint states in simulation
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Publishes the transforms defined in the robot_description to TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Launch RViz with the provided configuration file
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # List of all nodes to start when this launch file is run
    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return nodes_to_start
