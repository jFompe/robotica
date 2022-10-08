#!/usr/bin/env python3

import time
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
# from std_msgs.msg import String


class SimulationStopped(Exception):
    pass


class Pioneer_P3DX(Node):

    sonar_max = 1.0

    def __init__(self, robot_id):
        super().__init__('pioneer_p3dx_' + str(robot_id))
        self.current_time_updated = time.time()
        self.current_time = self.current_time_updated
        self.sonar3_reading = self.sonar_max
        self.sonar4_reading = self.sonar_max
        self.sonar5_reading = self.sonar_max
        self.sonar6_reading = self.sonar_max
        self.simulation_time = 0
        self.subscription = self.create_subscription(Float32, "/sonar3_"+robot_id, self.sonar3_callback, 10)
        self.subscription = self.create_subscription(Float32, "/sonar4_"+robot_id, self.sonar4_callback, 10)
        self.subscription = self.create_subscription(Float32, "/sonar5_"+robot_id, self.sonar5_callback, 10)
        self.subscription = self.create_subscription(Float32, "/sonar6_"+robot_id, self.sonar6_callback, 10)
        self.subscription = self.create_subscription(Float32, "/simulationTime_"+robot_id, self.simulation_time_callback, 10)
        self.subscription  # prevent unused variable warning
        self.left_motor_pub = self.create_publisher(Float32, "/leftMotorSpeed_"+robot_id, 10)
        self.right_motor_pub = self.create_publisher(Float32,"/rightMotorSpeed_"+robot_id, 10)
        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def sonar3_callback(self, msg):
        # self.get_logger().info('Sonar3: "%s"' % msg.data)
        self.sonar3_reading = msg.data

    def sonar4_callback(self, msg):
        # self.get_logger().info('Sonar4: "%s"' % msg.data)
        self.sonar4_reading = msg.data

    def sonar5_callback(self, msg):
        # self.get_logger().info('Sonar5: "%s"' % msg.data)
        self.sonar5_reading = msg.data

    def sonar6_callback(self, msg):
        # self.get_logger().info('Sonar6: "%s"' % msg.data)
        self.sonar6_reading = msg.data

    def simulation_time_callback(self, msg):
        # self.get_logger().info('Simulation time: "%f"' % msg.data)
        self.current_time_updated = time.time()
        self.simulation_time = msg.data

    def timer_callback(self):
        self.current_time = time.time()
        if (self.current_time - self.current_time_updated) > 9:
            # rclpy.shutdown()  # we didn't receive any sensor information for quite a while... we leave
            raise SimulationStopped

        if (self.sonar4_reading < 0.3) or (self.sonar5_reading < 0.3):
            # driving backwards
            desired_left_motor_speed  = +0.25
            desired_right_motor_speed = -0.55
        elif self.sonar3_reading < 0.3:
            # turning right
            desired_left_motor_speed  = +1.75
            desired_right_motor_speed = +1.15
        elif self.sonar6_reading < 0.2:
            # turning left
            desired_left_motor_speed  = +0.80
            desired_right_motor_speed = +1.55
        else:
            # going forward
            desired_left_motor_speed  = +1.95
            desired_right_motor_speed = +1.95

        # publish the motor speeds
        msg = Float32()
        msg.data = desired_left_motor_speed
        self.left_motor_pub.publish(msg)
        msg.data = desired_right_motor_speed
        self.right_motor_pub.publish(msg)


def main(args=None):
    if (len(sys.argv) < 2):
        print("Usage: ros2pioneer_p3dx.py robot-id")
    else:
        print('*** ros2pioneer_p3dx.py just started')
        rclpy.init(args=args)
        try:
            pioneer_p3dx = Pioneer_P3DX(sys.argv[1])
            rclpy.spin(pioneer_p3dx)
        except SimulationStopped:
            # print('*** Simulation stopped')
            pass
        finally:
            pioneer_p3dx.destroy_timer(pioneer_p3dx.timer)
            pioneer_p3dx.destroy_node()
            rclpy.shutdown()
            print('*** ros2pioneer_p3dx.py just ended')


if __name__ == '__main__':
    main()
