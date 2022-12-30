import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  PKG_NAME = 'mazebot'
  ROBOT_NAME = 'PioneerP3DX__158__'

  # Set the path to different files and folders.
  pkg_share = FindPackageShare(package=PKG_NAME).find(PKG_NAME)
  default_launch_dir = os.path.join(pkg_share, 'launch')
  urdf_model_path = os.path.join(pkg_share, f'models/{PKG_NAME}.urdf')
  robot_name_in_urdf = ROBOT_NAME
  rviz_config_file = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
  urdf = open(urdf_model_path).read()


  # Specify the actions

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    name='robot_state_publisher',
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': urdf}])

  # Publish the joint state values for the non-fixed joints in the URDF file.
  start_joint_state_publisher_cmd = Node(
    name='joint_state_publisher',
    package='joint_state_publisher',
    executable='joint_state_publisher',
    arguments=[urdf_model_path])

  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Add any actions
  ld.add_action(start_joint_state_publisher_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)

  return ld