from cmath import pi

from numpy import Infinity
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from random import randint
from enum import Enum

def clamp(value, minimum_value, maximum_value):
    return sorted((minimum_value, value, maximum_value))[1]

class State(Enum):
    MOVING = 0
    TURNING = 1

class VelocityController(Node):

    ticks_for_360 = 20

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')


        self.state = State.MOVING
        self.turning_ticks = 0
        self.min_range = 0
        self.angle_min_range = 0

    def timer_cb_moving(self):
        msg = Twist()

        self.get_logger().info(f'forward distance: {self.forward_distance}')

        if (self.forward_distance < 0.4):
            self.state = State.TURNING
            self.get_logger().info('turning...')
            self.turning_ticks = randint(0, self.ticks_for_360 // 2 - 1)
        else:
            msg.linear.x = 0.1

        self.publisher.publish(msg)
    
    def timer_cb_turning(self):
        msg = Twist()

        if (self.turning_ticks == 0):
            self.get_logger().info("moving...")
            self.state = State.MOVING
        else:
            if (self.angle_min_range > 1/2*pi and self.angle_min_range < 3/2*pi):
                self.turning_ticks -= 1
            msg.angular.z = 2*pi/self.ticks_for_360

        self.publisher.publish(msg)
    
    def timer_cb(self):
        if (self.state == State.MOVING):
            self.timer_cb_moving()
        elif (self.state == State.TURNING):
            self.timer_cb_turning()
        else:
            assert(False)
    
    def laser_cb(self, msg: LaserScan):
        min_range = Infinity
        min_range_index = 0
        for index in range(359):
            if (msg.ranges[index] < min_range):
                min_range = msg.ranges[index]
                min_range_index = index

        self.forward_distance = msg.ranges[0]
        self.min_range = min_range
        self.angle_min_range = min_range_index / 180 * pi

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




# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# class VelocityController(Node):

#     def __init__(self):
#         super().__init__('velocity_controller')
#         self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.forward_distance = 0
#         self.right_distance = 0
#         self.left_distance = 0
#         self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
#         self.create_timer(0.1, self.timer_cb)
#         self.get_logger().info('controller node started')
       
#     def timer_cb(self):
#         msg = Twist()
#         if self.forward_distance > 0.5 :
#             msg.linear.x=0.1
#             msg.linear.z = 0.0
#         elif self.right_distance > 0.5 :
#             msg.linear.x=0.0
#             msg.angular.z=-0.2
#         elif self.left_distance > 0.5 :
#             msg.linear.x= 0.0
#             msg.angular.z = 0.2        
#         self.publisher.publish(msg)
   
#     def laser_cb(self, msg):
#         self.forward_distance = msg.ranges[0]
#         self.right_distance = msg.ranges[270]
#         self.left_distance = msg.ranges[90]


# def main(args=None):
#     rclpy.init(args=args)

#     node = VelocityController()

#     rclpy.spin(node)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# class VelocityController(Node):

#     def __init__(self):
#         super().__init__('velocity_controller')
#         self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.forward_distance = 0
#         self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
#         self.create_timer(0.1, self.timer_cb)
#         self.get_logger().info('controller node started')
        
#     def timer_cb(self):
#         msg = Twist()
#         x = self.forward_distance - 0.3
#         z = 0.0
#         if x < 0.1:
#             x = x 
#         else: 
#             x=0.1

#         if x >= 0:
#             x = x
#         else: 
#             x=x
#             z=0.1

#         msg.angular.z = z
#         msg.linear.x = x
#         self.publisher.publish(msg)
    
#     def laser_cb(self, msg):
#         self.forward_distance = msg.ranges[0]



# def main(args=None):
#     rclpy.init(args=args)

#     node = VelocityController()

#     rclpy.spin(node)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()















# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import LaserScan

# class VelocityController(Node):

#     def init(self):
#         super().init('velocity_controller')
#         self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.forward_distance = 0
#         self.left_distance = 0
#         self.right_distance = 0
#         self.backward_distance = 0
#         self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
#         self.create_timer(0.1, self.timer_cb)
#         self.get_logger().info('controller node started')

#     def timer_cb(self):
#         msg = Twist()

#         if self.forward_distance > 0.1 and self.backward_distance>0.1 and self.left_distance>0.1 and self.right_distance>0.1:
#             msg.linear.x=0.2
#             msg.angular.z=0.2
#         else:
#             msg.angular.z =0.1
#             if self.forward_distance > 0.4 and self.backward_distance>0.4 and self.left_distance>0.1 and self.right_distance>0.1:
#                 msg.linear.x=0.1
#                 msg.angular.z=0.3

#         self.publisher.publish(msg)



#     def laser_cb(self, msg):
#         self.forward_distance = msg.ranges[0]
#         self.backward_distance = msg.ranges[360]
#         self.left_distance =msg.ranges[270]
#         self.right_distance = msg.ranges[90]



# def main(args=None):
#     rclpy.init(args=args)

#     node = VelocityController()

#     rclpy.spin(node)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()