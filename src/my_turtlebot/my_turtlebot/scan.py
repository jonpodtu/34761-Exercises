import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanNode(Node):
    def __init__(self):
        super().__init__('laser_scan_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS profile depth
        )
        self.subscription  # Prevent unused variable warning

    def scan_callback(self, msg):
        # Process LaserScan data here
        # For example, print the length of the ranges array
        self.get_logger().info(f"Number of measurements: {len(msg.ranges)}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
