from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ld = LaunchDescription()

    robot_urdf= Command(['xacro ',PathJoinSubstitution([FindPackageShare("two_joints_robot"),"urdf","two_joints_robot.xacro"])])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_urdf}
            ]
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',
                   PathJoinSubstitution([FindPackageShare("two_joints_robot"),"config","display.rviz"])],
        output='screen'
    )


    
    ld.add_action( joint_state_publisher_gui )
    ld.add_action( robot_state_publisher_node )
    ld.add_action(rviz_node)

    return ld