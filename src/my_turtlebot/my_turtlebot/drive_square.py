import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(5.0, self.move_square)
        self.step = 0

    def move_square(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        
        if self.step <= 7:
            msg.linear.x = 0.2 if self.step % 2 == 0 else 0.0
            msg.angular.z = (np.pi / 2) / 5 if self.step % 2 == 1 else 0.0
        
        self.publisher.publish(msg)
        self.step += 1

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()