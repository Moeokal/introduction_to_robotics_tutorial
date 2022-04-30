import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WatchdogNode(Node):

    def __init__(self):
        super().__init__('watchdog')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.twist = Twist()
        self.msg=''
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')

    def cmd_callback(self, msg):
        # this makes the turle go backwards
        # (just so you know its working)
        if self.msg=="start":
            # self.twist.linear.x = 0.0
            # self.twist.linear.y = 0.0
            # self.twist.linear.z = 0.0
            # self.twist.angular.x = 0.0
            # self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            # msg.linear.x = self.twist.linear.x
            # msg.linear.z = self.twist.angular.x
            # msg.linear.x = self.twist.linear.y
            # msg.linear.z = self.twist.angular.y
            # msg.linear.x = self.twist.linear.z
            msg.linear.x = -1 * msg.linear.x
            msg.angular.z = self.twist.angular.z
        elif self.msg=="stop":
            self.twist.linear.x =0.0
            self.twist.angular.z = 0.0
            msg.linear.x = self.twist.linear.x
            msg.angular.z = self.twist.angular.z
        else:
            # self.twist.linear.x = 1
            # self.twist.angular.z = 1
            msg.linear.x = -1 * msg.linear.x
            # self.get_logger().info(f'msg.linear.x: {msg.linear.x}')
        self.publisher.publish(msg)
        
    def controller_callback(self, msg):
        self.msg=msg.data
        self.get_logger().warn(f'The controller says I should {msg.data} the turtle ...')



def main(args=None):
    rclpy.init(args=args)

    node = WatchdogNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
