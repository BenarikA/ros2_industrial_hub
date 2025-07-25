# === LAUNCH CORE ===
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

# === ROS 2 LAUNCH EXTENSIONS ===
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

# === UTILITIES ===
from ament_index_python.packages import get_package_share_directory
import os
import xacro  # for xacro file parsing

def generate_launch_description():

    # === DECLARE USER-CONFIGURABLE ARGUMENTS ===
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="",
            description="model of the fanuc robot. ",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="False to use the standard mock of ros2",
            choices=["true", "false"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="Use if you have different controllers file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.5.100",
            description="the IP of the controlled robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz",
            default_value="false",
            description="If mock hardware, simulate using Gazebo ",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_headless",
            default_value="true",
            description="Use Gazebo in headless mode",
        )
    )

    # === RETURN FINAL LAUNCH DESCRIPTION ===
    return LaunchDescription( declared_arguments + [OpaqueFunction(function=launch_setup)] )

def launch_setup(context, *args, **kwargs):

    # === DESCRIPTION PACKAGE (assumes fanuc_description/urdf/<model>/<model>.xacro) ===
    description_package = "fanuc_description" 

    # === LAUNCH CONFIGURATION VARIABLES ===
    robot_type = LaunchConfiguration("robot_type")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    controllers_file = LaunchConfiguration("controllers_file")
    read_only = LaunchConfiguration("read_only")
    gz = LaunchConfiguration("gz")
    gz_headless = LaunchConfiguration("gz_headless")

    s_robot_type = robot_type.perform(context)
    
    # === Enable simulated clock if Gazebo is used ===
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('gz'))

    # === ROBOT DESCRIPTION (URDF from xacro) ===
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", s_robot_type, f"{s_robot_type}.xacro"]),
            " ", 
            "robot_type:=", robot_type,
            " ", 
            "use_mock_hardware:=", use_mock_hardware,
            " ", 
            "robot_ip:=", robot_ip,
            " ", 
            "read_only:=", read_only,
            " ", 
            "gz:=", gz
        ]
    )

    robot_description = {"robot_description": robot_description_content}    

    # === CONTROLLERS CONFIGURATION PATH ===
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("fanuc_control"),
            "config",
            controllers_file,
        ]
    )

    # === CORE NODES ===
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    # === SPAWN CONTROLLERS ===
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "-c", "/controller_manager"],
    )
    controller_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--inactive","forward_position_controller", "-c", "/controller_manager"],
    )

    # === MOVEIT CONFIGS (assumes <robot_type>_moveit_config package exists) ===
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(s_robot_type+'_moveit_config'),'launch', 'move_group.launch.py')])
    )

    # === RVIZ LAUNCH ===    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(s_robot_type+'_moveit_config'),'launch', 'moveit_rviz.launch.py')]))
    
    # === GAZEBO LAUNCH ===    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('fanuc_description'),'launch', 'gazebo.launch.py')]),
        launch_arguments={"headless" : gz_headless}.items(), 
        condition=IfCondition(gz))

    # === LIST OF NODES TO LAUNCH ===
    nodes_to_start = [
        set_use_sim_time,
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner_started,
        controller_spawner_inactive,
        robot_state_publisher_node,
        move_group,
        moveit_rviz,
        gazebo_launch
    ]

    return nodes_to_start