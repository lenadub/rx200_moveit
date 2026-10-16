import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class InputManagerNode(Node):
    def __init__(self):
        super().__init__('input_manager')
        self.get_logger().info("Input Manager Node has been started.")
        self.pub_ = self.create_publisher(Float32MultiArray, 'pick_place_request', 10)

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
    
    def send_request(self, pick_x, pick_y, pick_z, place_x, place_y, place_z):
        if self.validate_input(pick_x, pick_y, pick_z, place_x, place_y, place_z):
            request_msg = Float32MultiArray()
            request_msg.data = [float(pick_x), float(pick_y), float(pick_z), float(place_x), float(place_y), float(place_z)]
            self.get_logger().info(f"Sending request: {request_msg.data}")
            self.pub_.publish(request_msg)
        else:
            self.get_logger().error("Request not sent due to invalid input.")

def main(args = None):
    rclpy.init(args=args)
    node = InputManagerNode()
    #print(node.validate_input(float('inf'), -0.2, 0, 0.2, 0.2, 0))
    #node.send_request(float('nan'), -0.2, 0, 0.2, 0.2, 0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__" :
    main()