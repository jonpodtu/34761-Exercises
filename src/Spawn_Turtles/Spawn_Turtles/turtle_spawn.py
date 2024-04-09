# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import numpy as np
from geometry_msgs.msg import Twist, Vector3


from std_msgs.msg import String

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not availible, waiting again...')
        self.req = Spawn.Request()

    def send_request(self, name):
        self.req.x = random.uniform(2,8)
        self.req.y = random.uniform(2,8)
        self.req.theta = random.uniform(0, 2*np.pi)
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



class MinimalPublisher(Node):
    
    def __init__(self):
        super().__init__('turtle_mover')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.spin_turtle)
        self.publishers_ = []

    def create_turtle(self, turtle_name):
        self.publishers_.append(self.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10))

    def spin_turtle(self):
        msg = Twist()
        msg.linear = Vector3()
        msg.linear.x = 2.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular = Vector3()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 1.8

        for publisher_ in self.publishers_:
            publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_publisher = MinimalPublisher()

    for i in range(1, 11):
        turtle_name = 'turtle' + str(i)
        if i > 1:
            response = minimal_client.send_request(turtle_name)
        minimal_publisher.create_turtle(turtle_name=turtle_name)
        minimal_publisher.spin_turtle()

    rclpy.spin(minimal_publisher)

    minimal_client.destroy_node()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
