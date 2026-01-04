from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import Command,PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join

def generate_launch_description():
    ld= LaunchDescription()
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='False'
    )
    
    noxbot_description= Command(['xacro ',PathJoinSubstitution([FindPackageShare("noxbot_description_description"),"urdf","noxbot_description.xacro"])])
    noxbot_controllers= PathJoinSubstitution([FindPackageShare("noxbot_bringup"),"config","noxbot_controller.yaml"])
    gz_bridge_config= PathJoinSubstitution([FindPackageShare("noxbot_bringup"),"config","gazebo_bridge.yaml"])
    rviz_config=PathJoinSubstitution([FindPackageShare("noxbot_description_description"),"config","display.rviz"])

    show_gui = LaunchConfiguration('gui')
     
    robot_state_publisher_node= Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': noxbot_description}],
        output="screen"
    )
    #gazebo automatically start the controller manager
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
    )
    spawn_robot = TimerAction(
        period=5.0,  
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "noxbot_description",
                "-allow_renaming", "false",  # prevents "_1" duplicate
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_bridge_config}],
        output='screen'
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
    forward_command_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_command_controller"]
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
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner)
    ld.add_action(forward_command_controller_spawner)
    ld.add_action(rviz_node)
    ld.add_action(joint_state_publisher_gui_node)
    return ld
    