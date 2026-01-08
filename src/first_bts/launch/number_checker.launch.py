from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld=LaunchDescription()

    tree_xml_path=PathJoinSubstitution([FindPackageShare("first_bts"),"trees","number_checker.xml"])
    
    bt_node= Node(
        package="first_bts",
        executable="number_checker",
        output="screen",
        parameters=[{"tree_xml_file":tree_xml_path}]
    )
    ld.add_action(bt_node)
    return ld