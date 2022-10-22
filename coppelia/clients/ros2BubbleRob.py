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


class BubbleRob(Node):

    def __init__(self, left_motor_topic, right_motor_topic, sensor_topic, sim_time_topic):
        super().__init__('bubble_rob_' + str(int(time.time()) & 0x000fffff))

        # print('*** disable_signals:', self.disable_signals)
        # self.disable_signals = True
        # print('*** disable_signals:', self.disable_signals)

        self.current_time_updated = time.time()
        self.current_time = self.current_time_updated

        self.sensor_trigger = False
        self.simulation_time = 0
        self.drive_back_start_time = -99.0

        self.subscription = self.create_subscription(Bool, sensor_topic, self.sensor_callback, 10)
        self.subscription = self.create_subscription(Float32, sim_time_topic, self.simulation_time_callback, 10)
        self.subscription  # prevent unused variable warning

        self.left_motor_pub = self.create_publisher(Float32, left_motor_topic, 10)
        self.right_motor_pub = self.create_publisher(Float32, right_motor_topic, 10)

        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def sensor_callback(self, msg):
        # self.get_logger().info('Sensor: "%s"' % msg.data)
        self.current_time_updated = time.time()
        self.sensor_trigger = msg.data

    def simulation_time_callback(self, msg):
        # self.get_logger().info('Simulation time: "%f"' % msg.data)
        self.simulation_time = msg.data

    def timer_callback(self):
        self.current_time = time.time()
        if (self.current_time - self.current_time_updated) > 9:
            # rclpy.shutdown()  # we didn't receive any sensor information for quite a while... we leave
            raise SimulationStopped

        if (self.simulation_time - self.drive_back_start_time) < 3.0:
            # driving backwards while slightly turning
            desired_left_motor_speed  = -7.0 * 0.5
            desired_right_motor_speed = -7.0 * 0.25
        else:
            # going forward
            desired_left_motor_speed  = +7.0
            desired_right_motor_speed = +7.0

            if self.sensor_trigger:
                # we detected something and start the backward mode
                self.drive_back_start_time = self.simulation_time
            self.sensor_trigger = False

        # publish the motor speeds
        msg = Float32()
        msg.data = desired_left_motor_speed
        self.left_motor_pub.publish(msg)
        msg.data = desired_right_motor_speed
        self.right_motor_pub.publish(msg)

        # self.get_logger().info('Publishing: "%s"' % self.sensor_trigger)
        # self.get_logger().info('tic')


def main(args=None):
    print('*** ros2BubbleRob.py just started')

    if (len(sys.argv) < 5):
        print("*** Indicate following arguments: 'leftMotorTopic rightMotorTopic sensorTopic simulationTimeTopic'")

    else:
        left_motor_topic = "/" + sys.argv[1]
        right_motor_topic = "/" + sys.argv[2]
        sensor_topic = "/" + sys.argv[3]
        sim_time_topic = "/" + sys.argv[4]

        rclpy.init(args=args)

        try:
            bubble_rob = BubbleRob(left_motor_topic, right_motor_topic, sensor_topic, sim_time_topic)
            rclpy.spin(bubble_rob)
        except SimulationStopped:
            # print('Simulation stopped')
            pass
        finally:
            bubble_rob.destroy_timer(bubble_rob.timer)
            bubble_rob.destroy_node()
            rclpy.shutdown()
            print('*** ros2BubbleRob.py just ended')


if __name__ == '__main__':
    main()
