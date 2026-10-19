import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class InputManagerNode(Node):
    def __init__(self):
        super().__init__('input_manager')
        self.get_logger().info("Input Manager Node has been started.")
        self.robot_publisher = self.create_publisher(Float32MultiArray, 'pick_place_request', 10)
        self.status_subscriber = self.create_subscription(String, 'robot_status', self.status_callback, 10)
        self.inputs_subscriber = self.create_subscription(Float32MultiArray, 'user_inputs', self.user_inputs_callback, 10)
        self.gui_status_publisher = self.create_publisher(String, 'gui_status', 10)

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
            self.robot_publisher.publish(request_msg)
            self.get_logger().info(f"Request sent: {request_msg.data}")
            self.send_gui_status(f"Request sent: {request_msg.data}")
        else:
            self.get_logger().error("Request not sent due to invalid input.")
            self.send_gui_status("Request not sent due to invalid input.")

    def status_callback(self, msg):
        self.get_logger().info(f"Robot status: {msg.data}")
        self.send_gui_status(f"Robot status: {msg.data}")

    def user_inputs_callback(self, msg):
        coordinates = msg.data
        if len(coordinates) != 6:
            self.get_logger().warn(f"Expected 6 coordinates, received {len(coordinates)}.")
            self.send_gui_status(f"Expected 6 coordinates, received {len(coordinates)}.")
            return
        self.send_request(*coordinates)

    def send_gui_status(self, text):
        msg = String()
        msg.data = text
        self.gui_status_publisher.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = InputManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__" :
    main()