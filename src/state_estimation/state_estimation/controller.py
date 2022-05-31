import rclpy
import math
import numpy as np

from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan

def normalize(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)

def veclength(vector: np.ndarray) -> np.float64:
    return np.linalg.norm(vector)

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        
        self.n_positions_for_direction = 3
        self.last_positions = []
        
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')

    def calculate_direction(self):
        n = self.n_positions_for_direction
       
        diff_last = self.last_positions[-1] - self.last_positions[-2]
        diff_last_2 = self.last_positions[-2] - self.last_positions[-3]

        avg = np.average([diff_last, diff_last_2], axis=0)
        avg_norm = normalize(avg)
        #self.get_logger().info(f"avg: {avg} norm_avg: {avg_norm}")
        return avg_norm

        #direction = self.last_positions[-1] - self.last_positions[-2] 
        #return direction / np.linalg.norm(direction)

    def timer_cb(self):
        msg = Twist()

        msg.linear.x = 0.0

        if self.position is None:
            return

        if len(self.last_positions) < self.n_positions_for_direction:
            if self.forward_distance > 0.25:
                msg.linear.x = 0.1
        else:
            if self.forward_distance > 0.25:
                diff_to_goal = self.goal - self.position

                current_dir = self.calculate_direction()
                goal_dir = normalize(diff_to_goal)

                omega1 = math.atan2(current_dir[1], current_dir[0])
                omega2 = math.atan2(goal_dir[1], goal_dir[0])

                omega = omega2 - omega1

                msg.linear.x = min(max(veclength(diff_to_goal) - 0.2, 0.0), 0.05)
                msg.angular.z = min(max(omega*0.25, -0.2), 0.2)

        self.publisher.publish(msg)

    def goal_cb(self, msg):
        goal = np.array([msg.pose.position.x, msg.pose.position.y, 0.0])
        if (self.goal is None) or not np.allclose(goal, self.goal):
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]
        self.right_distance = msg.ranges[270]
        self.left_distance = msg.ranges[90]
        
    def position_cb(self, msg):
        pos2 = np.array([msg.point.x, msg.point.y, msg.point.z])
        self.position = pos2
        self.last_positions.append(pos2)

def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
