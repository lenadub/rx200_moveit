import rclpy
from rclpy.node import Node

class InputManagerNode(Node):
    def __init__(self):
        super().__init__('input_manager_node')

def main(args = None):
    rclpy.init(args=args)
    node = InputManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__" :
    main()