import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # Evaluate xacro once so controller_manager gets the exact URDF (with MobileBaseHwInterface, not GenericSystem)
    pkg_noxbot_desc = get_package_share_directory("noxbot_description_description")
    xacro_path = pkg_noxbot_desc + "/urdf/noxbot_description.xacro"
    result = subprocess.run(["xacro", xacro_path], capture_output=True, text=True, check=True)
    robot_description_str = result.stdout

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='False'
    )

    noxbot_controllers = PathJoinSubstitution([FindPackageShare("noxbot_bringup"), "config", "noxbot_controller.yaml"])
    rviz_config=PathJoinSubstitution([FindPackageShare("noxbot_description_description"),"config","display.rviz"])

    show_gui = LaunchConfiguration('gui')
     
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_str}],
        output="screen"
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[("/controller_manager/robot_description", "robot_description")],
        parameters=[noxbot_controllers, {"robot_description": robot_description_str}],
        output="screen"
    )
    joint_state_broadcaster_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )
    diff_drive_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"]
    )
    # forward_command_controller_spawner=Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["forward_command_controller"]
    # )
    arm_controller_spawner=Node(
         package="controller_manager",
        executable="spawner",
        arguments=["custom_arm_controller"]
    )
    rviz_node=Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config ]
    )
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gui_arg)
    # Delay controller_manager so it receives /robot_description from robot_state_publisher (transient_local QoS)
    ld.add_action(TimerAction(period=3.0, actions=[control_node]))
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner)
    #ld.add_action(forward_command_controller_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(rviz_node)
    ld.add_action(joint_state_publisher_gui_node)
    return ld
    