import math
import rclpy
from rclpy.node import Node

class InputManagerNode(Node):
    def __init__(self):
        super().__init__('input_manager')
        self.get_logger().info("Input Manager Node has been started.")

    def validate_input(self, pick_x, pick_y, pick_z, place_x, place_y, place_z):
        coordinates = (pick_x, pick_y, pick_z, place_x, place_y, place_z)
        for coordinate in coordinates:
            try:
                float(coordinate)
                if not math.isfinite(float(coordinate)):
                    self.get_logger().error(f"Invalid input: {coordinate} is not a finite number.")
                    return False
            except (ValueError, TypeError):
                self.get_logger().error(f"Invalid input: {coordinate} is not a valid float.")
                return False
        return True


def main(args = None):
    rclpy.init(args=args)
    node = InputManagerNode()
    #print(node.validate_input(float('inf'), -0.2, 0, 0.2, 0.2, 0))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__" :
    main()