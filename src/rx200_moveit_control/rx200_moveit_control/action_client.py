import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray, String
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion


class RX200MoveItActionClient(Node):

    def __init__(self):
        super().__init__('action_client')

        self._client = ActionClient(self, MoveGroup, '/move_action')
        # Additional MoveIt interfaces for short, straight-line movements.
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.arm_group = 'interbotix_arm'
        self.gripper_group = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'world'  # GitHub example uses the world frame
        self.gripper_joint = 'left_finger'
        self.gripper_start_value = 0.00

        # Adjust these to the physical object's height and a verified safe home pose.
        self.pick_clearance = 0.15
        self.place_clearance = 0.10
        self.home_pose = (0.2, 0.0, 0.2, 1.0)
        self.place_offset = 0.05

        self.busy = False
        self.sequence_steps = []
        self.step_index = 0

        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        self.request_sub = self.create_subscription(
            Float32MultiArray, 'pick_place_request', self.request_callback, 10
        )

        self.get_logger().info('RX200 MoveIt client started; waiting for MoveIt')
        # Do not block the executor waiting for MoveIt: keep the node responsive.
        self.server_timer = self.create_timer(1.0, self.check_moveit_server)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Robot status: {status}')

    def validate_position(self, x, y):
        distance = math.hypot(x ,y)
        min_reach = 0.275
        max_reach = 0.55

        # Allow for tiny float32 rounding at the experimentally tested limits.
        if distance < min_reach - 1e-5:
            return False
        if distance > max_reach + 1e-5:
            return False
        return True

    def check_moveit_server(self):
        if self._client.server_is_ready():
            self.server_timer.cancel()
            self.publish_status('IDLE')

    def request_callback(self, msg):
        if self.busy:
            self.get_logger().warning('Request ignored: robot is busy')
            return

        if not self._client.server_is_ready():
            self.get_logger().warning('Request ignored: MoveIt is unavailable')
            self.publish_status('FAILED')
            return

        if not self._cartesian_client.service_is_ready() or not self._execute_client.server_is_ready():
            self.get_logger().warning('Request ignored: Cartesian MoveIt interfaces unavailable')
            self.publish_status('FAILED')
            return

        if len(msg.data) != 8 or not all(math.isfinite(v) for v in msg.data):
            self.get_logger().warning('Expected 8 finite floats: pick xyzw, place xyzw')
            self.publish_status('FAILED')
            return

        pick_x, pick_y, pick_z, pick_w, place_x, place_y, place_z, place_w = msg.data

        # Keep the team's current w=+/-1 input contract for compatibility.
        # The target orientation is calculated from XYZ inside send_ee_pose().
        if not math.isclose(abs(pick_w), 1.0, abs_tol=1e-4) or not math.isclose(
            abs(place_w), 1.0, abs_tol=1e-4
        ):
            self.get_logger().warning('Invalid quaternion: w must be +1 or -1')
            self.publish_status('FAILED')
            return

        if not self.validate_position(pick_x, pick_y):
            self.get_logger().warning('Pick position outside workspace')
            self.publish_status('FAILED')
            return

        if not self.validate_position(place_x, place_y):
            self.get_logger().warning('Place position outside workspace')
            self.publish_status('FAILED')
            return

        self.sequence_steps = [
            ('gripper_start', self.gripper_start_value, 'set gripper start position'),
            ('arm', (pick_x, pick_y, pick_z + self.pick_clearance, pick_w), 'above pick'),
            ('gripper', True, 'open gripper'),
            ('cartesian', (pick_x, pick_y, pick_z+0.03, pick_w), 'lower to pick'),
            ('gripper', False, 'grasp object'),
            ('cartesian', (pick_x, pick_y, pick_z + self.pick_clearance, pick_w), 'lift object'),
            ('arm', (place_x, place_y, place_z + self.place_clearance, place_w), 'above place'),
            ('cartesian', (place_x, place_y, place_z+0.03, place_w), 'lower to place'),
            ('gripper', True, 'release object'),
            ('cartesian', (place_x, place_y, place_z + self.place_clearance, place_w), 'lift gripper'),
            ('arm', self.home_pose, 'return home'),
        ]
        self.step_index = 0
        self.busy = True
        self.publish_status('BUSY')
        self.send_next_step()

    def send_next_step(self):
        if self.step_index >= len(self.sequence_steps):
            self.busy = False
            self.publish_status('SUCCESS')
            self.publish_status('IDLE')
            return

        kind, value, label = self.sequence_steps[self.step_index]
        self.get_logger().info(f'Step {self.step_index + 1}: {label}')
        try:
            if kind == 'arm':
                self.send_ee_pose(*value)
            elif kind == 'cartesian':
                self.send_ee_pose(*value, cartesian=True)
            elif kind == 'gripper_start':
                self.send_gr_pose(position = value)
            else:
                self.send_gr_pose(value)
        except Exception as exc:
            self.fail_sequence(f'Could not send {label}: {exc}')

    def fail_sequence(self, reason):
        self.get_logger().error(reason)
        self.sequence_steps = []
        self.busy = False
        self.publish_status('FAILED')
        # The robot may be in an unsafe/unexpected pose; do not auto-home on failure.

    def send_ee_pose(self, x, y, z, w=1.0, cartesian=False):
        # MoveGroup goal structure from the lecturer, pose calculation from GitHub.
        p = PoseStamped()
        p.header.frame_id = self.ref_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        # GitHub send_pose() orientation logic, using this node's existing w input.
        yaw = math.atan2(y, x)
        roll = 0.0
        distance_from_base = math.hypot(x, y)
        if w <= 0.2:
            pitch = 0.53
        elif distance_from_base < 0.087 and z < 0.19:
            pitch = 1.57
        else:
            pitch = 0.0

        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, spitch = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        p.pose.orientation = Quaternion(
            x=sr * cp * cy - cr * spitch * sy,
            y=cr * spitch * cy + sr * cp * sy,
            z=cr * cp * sy - sr * spitch * cy,
            w=cr * cp * cy + sr * spitch * sy,
        )
        self.get_logger().info(
            f'Pose target: ({x:.3f}, {y:.3f}, {z:.3f}), '
            f'pitch={math.degrees(pitch):.1f} deg, '
            f'yaw={math.degrees(yaw):.1f} deg, frame={self.ref_frame}'
        )

        if cartesian:
            self.plan_cartesian_path(p)
            return

        req = MotionPlanRequest()
        req.group_name = self.arm_group
        req.allowed_planning_time = 7.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.ref_frame
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [p.pose]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.ref_frame
        oc.link_name = self.ee_link
        oc.orientation = p.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 0.5

        gc = Constraints()
        gc.position_constraints = [pc]
        gc.orientation_constraints = [oc]
        req.goal_constraints = [gc]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.look_around = False

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def plan_cartesian_path(self, pose_stamped):
        # Compute a straight end-effector path from the CURRENT arm state.
        # This is only used for the four short pick/place approach/retreat steps.
        request = GetCartesianPath.Request()
        request.header = pose_stamped.header
        request.start_state.is_diff = True
        request.group_name = self.arm_group
        request.link_name = self.ee_link
        request.waypoints = [pose_stamped.pose]
        request.max_step = 0.005          # 5 mm interpolation resolution
        request.jump_threshold = 1.5      # Reject unusually large joint jumps
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.10
        request.max_acceleration_scaling_factor = 0.10

        future = self._cartesian_client.call_async(request)
        future.add_done_callback(self._cartesian_path_cb)

    def _cartesian_path_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(
                f'Cartesian path fraction: {result.fraction:.3f}'
            )
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                self.fail_sequence(
                    f'Cartesian planning failed: code {result.error_code.val}'
                )
                return

            # Never execute a partial approach or lift.
            if result.fraction < 0.999 or not result.solution.joint_trajectory.points:
                self.fail_sequence(
                    f'Incomplete Cartesian path ({result.fraction * 100:.1f}%)'
                )
                return

            if not self._execute_client.server_is_ready():
                self.fail_sequence('ExecuteTrajectory action unavailable')
                return

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = result.solution
            execute_future = self._execute_client.send_goal_async(goal)
            # Reuse the original result callback to advance the step index only
            # AFTER trajectory execution has completed successfully.
            execute_future.add_done_callback(self._goal_response_cb)
        except Exception as exc:
            self.fail_sequence(f'Cartesian path error: {exc}')

    def send_gr_pose(self, open=True, position = None):
        # Based on the lecturer's joint-constrained gripper implementation.
        req = MotionPlanRequest()
        req.group_name = self.gripper_group
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        # Gripper positions
        GRIPPER_OPEN = 0.045
        GRIPPER_CLOSED = 0.031

        

        if position is not None:
            jc.position = float(position)
        else:
            jc.position = GRIPPER_OPEN if open else GRIPPER_CLOSED

        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0

        self.get_logger().info(
            f'Gripper target position: {jc.position}'
    )

        gc = Constraints()
        gc.joint_constraints = [jc]
        req.goal_constraints = [gc]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            handle_goal = future.result()
            if not handle_goal.accepted:
                self.fail_sequence('Goal rejected by MoveIt')
                return
            self.get_logger().info('Goal accepted')
            result_future = handle_goal.get_result_async()
            result_future.add_done_callback(self._result_cb)
        except Exception as exc:
            self.fail_sequence(f'Goal response error: {exc}')

    def _feedback_cb(self, feedback_msg):
        self.get_logger().info(f'Current State: {feedback_msg.feedback.state}')

    def _result_cb(self, future):
        try:
            wrapped_result = future.result()
            code = wrapped_result.result.error_code.val
            self.get_logger().info(f'[RESULTS] Result Code: {code}')
            if code != MoveItErrorCodes.SUCCESS:
                self.fail_sequence(f'MoveIt failed at step {self.step_index + 1}: code {code}')
                return
            self.step_index += 1
            self.send_next_step()
        except Exception as exc:
            self.fail_sequence(f'Result callback error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = RX200MoveItActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
