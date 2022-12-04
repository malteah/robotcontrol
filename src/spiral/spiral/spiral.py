import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('spiral')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()

        msg.linear.x = float(self.i/10)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.0

        
        #msg.data = "{'linear':{'x':1.0,'y':0.0,'z':0.0},'angular':{'x':0.0,'y':0.0,'z':1.0}}"
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    spiral = MinimalPublisher()

    rclpy.spin(spiral)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spiral.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
