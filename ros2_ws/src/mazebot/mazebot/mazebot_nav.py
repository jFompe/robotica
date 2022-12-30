

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult



def main():

  rclpy.init()

  goal_pose = MazeStarter().wait_for_goal()

  navigator = BasicNavigator()

  navigator.waitUntilNav2Active()

  navigator.goToPose(goal_pose)

  while not navigator.isNavComplete():

    feedback = navigator.getFeedback()
    print(f'Distance reimaining: {feedback.distance_remaining} meters')

    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
      navigator.cancelNav()

    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
      goal_pose.pose.position.x = -3.0
      navigator.goToPose(goal_pose)


  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
      print('Goal succeeded!')
  elif result == NavigationResult.CANCELED:
      print('Goal was canceled!')
  elif result == NavigationResult.FAILED:
      print('Goal failed!')
  else:
      print('Goal has an invalid return status!')

  # Shut down the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()

  exit(0)


class MazeStarter(Node):

  def __init__(self, id: str) -> None:
    super().__init__(f'mazebot_{id}')
    self.goal_pose = None
    self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
    self.create_subscription(Point, '/finish_line', self.set_finish_line_callback, 10)

  def set_finish_line_callback(self, msg):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = 0
    goal_pose.pose.position.x = msg.x
    goal_pose.pose.position.y = msg.y
    goal_pose.pose.position.z = msg.z
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    self.goal_pose = goal_pose

  def wait_for_goal(self):
    while self.goal_pose is None:
      pass
    self.goal_pub.publish(self.goal_pose)
    return self.goal_pose



if __name__ == '__main__':
  main()
    