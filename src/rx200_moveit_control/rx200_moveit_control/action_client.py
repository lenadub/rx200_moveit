import rclpy    
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32MultiArray, String

class rx200ActionClient(Node):
    def __init__(self):
        super().__init__('rx200_action_client')

        self.get_logger().info('RX200 Action Client started')

        self._client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )    

        while rclpy.ok() and not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('waiting for MoveIt Action Server')

        if rclpy.ok():
            self.get_logger().info('MoveIt Action Server connected')

        # MoveIt planning groups
        self.arm_group = 'interbotix_arm'
        self.gripper_group = 'interbotix_gripper'

        # Robot coordinate frames
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'

        # Gripper joint
        self.gripper_joint = 'left_finger'

        self.request_subscriber = self.create_subscription(
            Float32MultiArray, 
            'pick_place_request', 
            self.pick_place_callback,
            10
        )

    def send_ee_pose(self, x, y, z, w=1.0):
        p = PoseStamped()
        p.header.frame_id = self.base_link
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation = Quaternion(
            x = 0.0,
            y = 0.0,
            z = 0.0,
            w = w
        )
        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]

        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [p.pose]

        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link

        oc.orientation = p.pose.orientation

        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        gc = Constraints()

        gc.position_constraints = [pc]
        gc.orientation_constraints = [oc]

        req.goal_constraints = [gc]

        goal = MoveGroup.Goal

        goal.request = req

        goal.planning_options.plan= False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        future = self._client.send_goal_async(
            goal, 
            feedback_callback=self._feedback_cb
        )

        future.add_done_callback(self._goal_response_cb)

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.gripper_group

        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1
        #
        jc = JointConstraint()
        jc.joint_name - self.gripper_group
        jc.position = 0.35 if open else 0.0
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        gc = Constraints()
        gc.joint_constraints = [jc]
        req.goal_constraints = [gc]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_respone_cb)


    def _goal_respone_cb(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal Rejected by MoveIt')
            return
        self.get_logger().info('Goal Accepted by Moveit')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _goal_response_cb(self, feedback_msg):
        state = feedback_msg.feedback_state
        self.get_logger().info(f'Current MoveIt State: {state}')

    def _result_cb(self, future):
        result = future.result().result
        code = result.error_code.val
        self.get_logger().info(f'MoveIt Result Code: {code}')

        if code == 1:
            self.get_logger().info('Robot Movement Successful')

        else:
            self.get_logger().error('Robot Movement Failed')

    def pick_place_callback(self,msg):
        if len(msg.data) != 6:
            self.get_logger().error('Invalid Request: Expected 6 Coords')
            return

        pick_x, pick_y, pick_z, place_x, place_y, place_z = msg.data

        self.get_logger().info(f'Pick:({pick_x}, {pick_y}, {pick_z})')
        self.get_logger().info(f'Place: ({place_x}, {place_y}, {place_z})')

        




def main(args=None):
    rclpy.init(args=args)

    node = rx200ActionClient()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()