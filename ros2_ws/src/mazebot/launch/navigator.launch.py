from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

  maze_navigator = Node(
    package="mazebot",
    executable="navigator"
  )

  return LaunchDescription([
    maze_navigator
  ])