import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-32:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def iterate(self, estimate):
        R = np.zeros(len(self.anchor_ranges))
        delta_R = np.zeros((len(self.anchor_ranges), 3))
        i = 0
        for a in self.anchor_ranges:
            anchor_pos = np.array([a.anchor.x, a.anchor.y, a.anchor.z])
            
            diff = estimate - anchor_pos
            
            R[i] = a.range - np.linalg.norm(diff)
            delta_R[i, :] = -diff / np.linalg.norm(diff)
            i += 1

        new_x = estimate - np.linalg.pinv(delta_R) @ R
        return new_x

    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0
        
        # YOUR CODE GOES HERE:

        estimate = np.array([0.1, 0.1, 0.1])
        for i in range(30):
            estimate = self.iterate(estimate)

        self.get_logger().info(f"estimate: {estimate}")

        return estimate[0], estimate[1], estimate[2]


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
