import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class SimulationStopped(Exception):
    pass



class Mazebot(Node):

    SONAR_MAX = 1.0
    NUM_SONARS = 16
    TIMER_PERIOD = 0.05 # seconds

    def __init__(self, id: str) -> None:
        super().__init__(f'mazebot_{id}')
        self.current_time_updated = time.time()
        self.current_time = self.current_time_updated
        self.sonar_readings = [self.SONAR_MAX] * self.NUM_SONARS
        self.simulation_time = 0
        self.simulation_time_sub = self.create_subscription(Float32, f'simulationTime_{id}', self.simulation_time_callabck, 10)
        self.sonar_subs = [ self.create_subscription(Float32, f'/sonar{i}_{id}', lambda m: self.sonar_callback(i, m), 10) for i in range(self.NUM_SONARS) ]
        self.left_motor_pub = self.create_publisher(Float32, f'leftMotorSpeed_{id}', 10)
        self.right_motor_pub = self.create_publisher(Float32, f'rightMotorSpeed_{id}', 10)
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

    def sonar_callback(self, sonar_id: int, msg: dict) -> None:
        self.get_logger().info(f'Sonar{sonar_id}: {msg.data}')
        self.sonar_readings[sonar_id] = msg.data

    def simulation_time_callback(self, msg: dict) -> None:
        self.current_time_updated = time.time()
        self.simulation_time = msg.data

    def compute_speeds(self) -> tuple:
        pass

    def move_motors(self) -> None:
        left, right = self.compute_speeds()

        # Publish motor speeds
        msg = Float32()
        msg.data = left
        self.left_motor_pub.publish(left)
        msg.data = right
        self.right_motor_pub.publish(right)


    def timer_callback(self) -> None:
        self.current_time = time.time()
        if (self.current_time - self.current_time_updated) > 9:
            # rclpy.shutdown()  # we didn't receive any sensor information for quite a while... we leave
            raise SimulationStopped

        self.move_motors()



def main(args=None):
    if (len(sys.argv) < 2):
        print("Usage: mazebot.py robot-id")
        return -1

    print('*** mazebot.py just started')
    rclpy.init(args=args)
    try:
        mazebot,   = Mazebot(sys.argv[1])
        rclpy.spin(mazebot)
    except SimulationStopped:
        # print('*** Simulation stopped')
        pass
    finally:
        mazebot.destroy_timer(mazebot.timer)
        mazebot.destroy_node()
        rclpy.shutdown()
        print('*** mazebot.py just ended')


if __name__ == '__main__':
    main()
