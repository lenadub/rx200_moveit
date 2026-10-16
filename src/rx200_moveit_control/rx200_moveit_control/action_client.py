import rclpy    
from rclpy.node import Node

class rx200ActionClient(Node):
    def __init__(self):
        super().__init__('rx200_action_client')

        self.get_logger().info('RX200 Action Client started')


def main(args=None):
    rclpy.init(args=args)

    node = rx200ActionClient()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()