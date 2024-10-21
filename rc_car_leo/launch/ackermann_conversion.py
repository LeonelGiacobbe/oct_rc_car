import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

class AckermannConverter(Node):
    def __init__(self):
        super().__init__('ackermann_converter')
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 0)
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 0)
        self.wheelbase = 0.33  # Wheelbase of the vehicle in meters
        self.track_width = 3.81  # Track width of the vehicle in meters

    def twist_callback(self, msg):
        linear_speed = -msg.linear.x * 0.25  # Scale linear speed
        angular_speed = msg.angular.z

        # Compute the steering angle using wheelbase
        steering_angle = angular_speed / (self.wheelbase / 2)

        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = - linear_speed
        ack_msg.drive.steering_angle = - steering_angle
        ack_msg.drive.acceleration = 0.8  # Constant acceleration
        self.pub.publish(ack_msg)

def main(args=None):
    rclpy.init(args=args)
    ackermann_converter = AckermannConverter()

    try:
        rclpy.spin(ackermann_converter)
    except KeyboardInterrupt:
        ackermann_converter.get_logger().info('Node was stopped cleanly')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from geometry_msgs.msg import Twist
# from ackermann_msgs.msg import AckermannDriveStamped

# def twist_callback(msg):
# 	linear_speed = msg.linear.x * 0.5 # 0.5 * multiplier. m/2
# 	angular_speed = msg.angular.z

# 	steering_angle = - angular_speed / (wheelbase / 2) # radians
	
# 	ack_msg = AckermannDriveStamped()
# 	ack_msg.drive.speed = linear_speed
# 	ack_msg.drive.steering_angle = steering_angle
# 	ack_msg.drive.acceleration = 0.8
# 	pub.publish(ack_msg)


# def main(args=None):
# 	rclpy.init(args=args)
# 	conversion_node = rclpy.create_node('ackermann_conversion')
	
# 	global pub
# 	pub = conversion_node.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
# 	sub = conversion_node.create_subscription(Twist, 'cmd_vel', twist_callback, 10)

# 	global wheelbase, track_width

# 	wheelbase = 0.1
# 	track_width = 3.81

# 	rclpy.spin(conversion_node)
# 	rclpy.shutdown()

# if __name__ == '__main__':
# 	main()
