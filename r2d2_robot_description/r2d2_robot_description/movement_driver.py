import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

internal_topic_rviz = '/diff_drive_controller/cmd_vel_unstamped'
internal_topic_gazebo = '/r2d2/diff_drive_controller/cmd_vel_unstamped'
external_topic = 'movement_command'
queue_size = 10

class MovementDriver(Node):

    def __init__(self, sim):

        super().__init__('movement_driver')

        self.sub = self.create_subscription(
            Twist,
            external_topic,
            self.handle_callback,
            queue_size
        )

        if sim == 'rviz':
            self.pub = self.create_publisher(
                Twist,
                internal_topic_rviz,
                queue_size
            )
        else:
            self.pub = self.create_publisher(
                Twist,
                internal_topic_gazebo,
                queue_size
            )

    def handle_callback(self, data):

        self.pub.publish(data)

def main(args=None):

    try:
        rclpy.init(args=args)

        sim = sys.argv[1]

        movement_driver = MovementDriver(sim)

        movement_driver.get_logger().info(
            'Movement Driver Node Initalized...'
        )

        rclpy.spin(movement_driver)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()