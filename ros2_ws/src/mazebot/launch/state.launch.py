import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

  PKG_NAME = 'mazebot'

  pkg_share = FindPackageShare(package=PKG_NAME).find(PKG_NAME)
  urdf_model_path = os.path.join(pkg_share, f'models/{PKG_NAME}.urdf')
  with open(urdf_model_path, 'r') as urdf_f:
    robot_desc = urdf_f.read()

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    name='robot_state_publisher',
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    arguments=[urdf_model_path])

  # Publish the joint state values for the non-fixed joints in the URDF file.
  # TODO REMOVE ???
  start_joint_state_publisher_cmd = Node(
    name='joint_state_publisher',
    package='joint_state_publisher',
    executable='joint_state_publisher',
    arguments=[urdf_model_path])

  return LaunchDescription([
    start_robot_state_publisher_cmd,
    start_joint_state_publisher_cmd
  ])