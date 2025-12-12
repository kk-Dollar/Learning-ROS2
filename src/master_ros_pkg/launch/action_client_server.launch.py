import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='master_ros_pkg',
      executable='action_server',
      name='action_server',
      output='screen',
    ),
    Node(
      package='master_ros_pkg',
      executable='action_client',
      name='action_client',
      output='screen'
    ),
  ])