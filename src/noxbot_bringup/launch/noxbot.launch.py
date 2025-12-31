from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import Command,PathJoinSubstitution

def generate_launch_description():
    ld= LaunchDescription()
    noxbot_description= Command(['xacro ',PathJoinSubstitution([FindPackageShare("noxbot_description_description"),"urdf","noxbot_description.xacro"])])
    noxbot_controllers= PathJoinSubstitution([FindPackageShare("noxbot_bringup"),"config","noxbot_controller.yaml"])
    rviz_config=PathJoinSubstitution([FindPackageShare("noxbot_description_description"),"config","display.rviz"])
     
    robot_state_publisher_node= Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': noxbot_description}],
        output="screen"
    )
    control_node=Node(
        package="controller_manager",
        executable="ros2_control_node",
        remappings=[("/controller_manager/robot_description","robot_description")],
        parameters=[noxbot_controllers,
        {'robot_description': noxbot_description }],
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
    rviz_node=Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config ]
    )
    ld.add_action(robot_state_publisher_node)
    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner)
    ld.add_action(rviz_node)
    return ld
    