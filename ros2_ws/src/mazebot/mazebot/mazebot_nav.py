import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():

  rclpy.init()

  navigator = MazeNavigator()

  rclpy.spin(navigator)

  navigator.waitUntilNav2Active()

  while not navigator.isNavComplete():

    feedback = navigator.getFeedback()
    print(f'Distance reimaining: {feedback.distance_remaining} meters')

    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
      navigator.cancelNav()

    '''
    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
      goal_pose.pose.position.x = -3.0
      navigator.goToPose(goal_pose)
    '''


  result = navigator.getResult()
  if result == TaskResult.SUCCEEDED:
      print('Goal succeeded!')
  elif result == TaskResult.CANCELED:
      print('Goal was canceled!')
  elif result == TaskResult.FAILED:
      print('Goal failed!')
  else:
      print('Goal has an invalid return status!')

  # Shut down the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()
  navigator.destroy_node()
  rclpy.shutdown()

  exit(0)


class MazeNavigator(BasicNavigator):

  def __init__(self) -> None:
    super().__init__()
    self.get_logger().info('Creating MazeNavigator')
    self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

  def goal_pose_callback(self, msg):
    self.get_logger().info('Received pose message: {msg}')
    self.goToPose(msg.pose)
    self.get_logger().info('Mazebot ordered to go to goal pose')
