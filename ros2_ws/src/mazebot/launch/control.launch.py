import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart



def generate_launch_description():

  PKG_NAME = 'mazebot'

  pkg_share = FindPackageShare(package=PKG_NAME).find(PKG_NAME)
  urdf_model_path = os.path.join(pkg_share, f'models/{PKG_NAME}.urdf')
  controller_config_file = os.path.join(pkg_share, 'config/diff_drive_controller.yml')

  with open(urdf_model_path, 'r') as urdf_f:
    robot_desc = urdf_f.read()


  controller_manager = Node(
      package="controller_manager",
      executable="ros2_control_node",
      parameters=[{'robot_description': robot_desc}, controller_config_file]
  )


  delayed_controller_manager = TimerAction(period=20.0, actions=[controller_manager])

  diff_drive_spawner = Node(
      package="controller_manager",
      executable="spawner.py",
      arguments=["diff_cont"],
  )

  delayed_diff_drive_spawner = RegisterEventHandler(
      event_handler=OnProcessStart(
          target_action=controller_manager,
          on_start=[diff_drive_spawner],
      )
  )

  joint_broad_spawner = Node(
      package="controller_manager",
      executable="spawner.py",
      arguments=["joint_broad"],
  )

  delayed_joint_broad_spawner = RegisterEventHandler(
      event_handler=OnProcessStart(
          target_action=controller_manager,
          on_start=[joint_broad_spawner],
      )
  )

  return LaunchDescription([
    delayed_controller_manager,
    delayed_diff_drive_spawner,
    delayed_joint_broad_spawner
  ])