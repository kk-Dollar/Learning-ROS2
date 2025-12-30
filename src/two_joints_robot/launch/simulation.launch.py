from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess,IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration,Command
from ament_index_python.packages import get_package_share_directory
import os
import time

def generate_launch_description():
    ld=LaunchDescription()

    robot_urdf=Command(['xacro ',PathJoinSubstitution([FindPackageShare("two_joints_robot"),"urdf","two_joints_robot.xacro"])])

    robot_state_publisher_node=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {'robot_description':robot_urdf}
        ]

    )
    spawn_node= Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            '-name', 'two_joints_robot',
            '-topic', '/robot_description'
        ],
        output="screen"
    )
    ignition_gazebo_node=IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"),"launch","gz_sim.launch.py"])),
    launch_arguments=[('gz_args',['-r -v 4 empty.sdf'])])

    load_joint_state_broadcaster= ExecuteProcess(
        cmd=['ros2','control','load_controller','--set-state','active','joint_state_broadcaster'],
        output='screen'
    )
    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'position_control'],
        output='screen'
    )


    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'velocity_control'],
        output='screen'
    )
    
    ld.add_action( robot_state_publisher_node )
    ld.add_action( spawn_node )
    ld.add_action( ignition_gazebo_node )
    ld.add_action( load_position_controller )
    ld.add_action( load_velocity_controller )
    ld.add_action( load_joint_state_broadcaster )
   
   
    

    return ld