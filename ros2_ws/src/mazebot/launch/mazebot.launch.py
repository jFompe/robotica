import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

  PKG_NAME = 'mazebot'

  # Set the path to different files and folders.
  pkg_share = FindPackageShare(package=PKG_NAME).find(PKG_NAME)
  rviz_config_file = os.path.join(pkg_share, 'rviz/rviz_config.rviz')
  
  state = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(PKG_NAME), 'launch', 'state.launch.py'
                )])
  )

  control = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(PKG_NAME),'launch','control.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )


  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  
  # Create the launch description and populate
  return LaunchDescription([
    state,
    start_rviz_cmd
    # control
  ])