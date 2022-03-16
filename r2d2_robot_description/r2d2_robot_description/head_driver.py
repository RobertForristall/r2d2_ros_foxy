import rclpy
from rclpy.node import Node
from std_msgs.msg import (Float64, Float64MultiArray)
import sys

internal_topic_rviz = '/head_velocity_controller/commands'
internal_topic_gazebo = '/r2d2/head_velocity_controller/commands'
external_topic = 'head_command'
queue_size = 10

class HeadDriver(Node):

    def __init__(self, sim):

        super().__init__("head_driver")

        self.sub = self.create_subscription(
            Float64,
            external_topic,
            self.handle_callback,
            queue_size
        )

        if sim == 'rviz':
            self.pub = self.create_publisher(
                Float64MultiArray,
                internal_topic_rviz,
                queue_size
            )
        else:
            self.pub = self.create_publisher(
                Float64MultiArray,
                internal_topic_gazebo,
                queue_size
            )
    
    def handle_callback(self, data):

        msg = Float64MultiArray()
        msg.data.append(data.data)
        self.pub.publish(msg)
    

def main(args=None):

    try:
        rclpy.init(args=args)

        sim = sys.argv[1]

        driver_node = HeadDriver(sim)

        driver_node.get_logger().info(
            'Head Driver Node Initalized...'
        )

        rclpy.spin(driver_node)
    
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
