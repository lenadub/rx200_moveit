import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
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
#connecting to the MoveIt action server 
        self._client = ActionClient(self, MoveGroup, '/move_action')
#Additional clients for calculating and executing straightline movements using Cartesian path planning
        self._cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

#Reading the actual end effector pose before each Cartesian movement
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

#Identifying the rx200 joints
        self.arm_group = 'interbotix_arm'
        self.gripper_group = 'interbotix_gripper'
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.ref_frame = 'world'  # GitHub example uses the world frame
        self.gripper_joint = 'left_finger'
        self.gripper_start_value = 0.00

#Defining a safe distance above the pick and place coords to showcase straightline movement and prevent sliding
        self.pick_clearance = 0.15
        self.place_clearance = 0.15
#Defining a known home position for the rx200 to move to at the beginning and end of each movement
        self.home_pose = (0.2, 0.0, 0.2, 1.0)

#Flags to prevent overlapping tasks
        self.busy = False
        self.returning_home = False
        self.sequence_steps = []
        self.step_index = 0

#Creating a timer to allow the rx200 to settle in between movements within a task - preventing position mismatch
        self.settle_timer = None
#Allow standard MoveIt planning if Cartesian planning fails
        self.allow_cartesian_fallback = True  # Do not execute unreviewed joint-space fallback

#Storing the most recently received joint measurements
        self.latest_joint_state = None

#Subscribing to the actual rx200 joint readings for Cartesian path planning
        self.joint_state_sub = self.create_subscription(
           JointState,
            '/rx200/joint_states',
            self.joint_state_callback,
            10
        )
#Creating a topic which the action client publishes to, detailing the activity status of the rx200
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
#Subscribing to pick and place topic to receive inputted coords
        self.request_sub = self.create_subscription(
            Float32MultiArray, 'pick_place_request', self.request_callback, 10
        )
#Checking if MoveIt server is available 
        self.get_logger().info('RX200 MoveIt client started; waiting for MoveIt')
        self.server_timer = self.create_timer(1.0, self.check_moveit_server)

#Publishes the rx200 status to the robot_status topic and logs it in the terminal
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Robot status: {status}')

#Checks if inputted coords lie within a measured 'safe distance' - this was measured by experimentation to be limits of reliable safe movement
#May be redundant with GUI validation but was written early in project
    def validate_position(self, x, y):
        distance = math.hypot(x ,y)
        min_reach = 0.275
        max_reach = 0.55
        if distance < min_reach :
            return False
        if distance > max_reach:
            return False
        return True
#Checking for MoveIt server and publishing that rx200 is ready for a task
    def check_moveit_server(self):
        if self._client.server_is_ready():
            self.server_timer.cancel()
            self.publish_status('IDLE')

#Publishes when system rejects and input and explains why
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

        if len(msg.data) != 6 or not all(math.isfinite(v) for v in msg.data):
            self.get_logger().warning('Expected 6 finite floats: pick xyz, place xyz')
            self.publish_status('FAILED')
            return

        pick_x, pick_y, pick_z, place_x, place_y, place_z = msg.data

        if not self.validate_position(pick_x, pick_y):
            self.get_logger().warning('Pick position outside workspace')
            self.publish_status('FAILED')
            return

        if not self.validate_position(place_x, place_y):
            self.get_logger().warning('Place position outside workspace')
            self.publish_status('FAILED')
            return
#Defines the sequence of movements the rx200 will take to complete the task
#To prevent possible collision with the object being picked and placed - before the pick operation and after the place operation, the rx200 will move to a predefined height 
#directly above the locations
#Different kinds are defined for different desired movement types
        self.sequence_steps = [
            ('arm', self.home_pose, 'return home'),
            ('gripper_start', self.gripper_start_value, 'set gripper start position'),
            ('arm', (pick_x, pick_y, pick_z + self.pick_clearance), 'above pick'),
            ('gripper', True, 'open gripper'),
            ('cartesian', (pick_x, pick_y, pick_z+0.045), 'lower to pick'),
            ('gripper', False, 'grasp object'),
            ('cartesian', (pick_x, pick_y, pick_z + self.pick_clearance), 'lift object'),
            ('arm', (place_x, place_y, place_z + self.place_clearance), 'above place'),
            ('cartesian', (place_x, place_y, place_z+0.045), 'lower to place'),
            ('gripper', True, 'release object'),
            ('cartesian', (place_x, place_y, place_z + self.place_clearance), 'lift gripper'),
            ('arm', self.home_pose, 'return home'),
        ]
        self.step_index = 0
        self.busy = True

#Publishes that the rx200 is performing a task
        self.publish_status('BUSY')
        self.send_next_step()

#Reads the next step and sends the correct command 
    def send_next_step(self):
#if the step index is >= the length of the sequence list, the task is complete and publishes as such
        if self.step_index >= len(self.sequence_steps):
            self.busy = False

            if self.returning_home:
                self.returning_home = False
                self.publish_status('IDLE')
            else:
                self.publish_status('SUCCESS')
                self.publish_status('IDLE')

            return

#Extracts the movement type, target and label
        kind, value, label = self.sequence_steps[self.step_index]
        self.get_logger().info(f'Step {self.step_index + 1}: {label}')

#Sends the target through relevant normal MoveIt, Cartesian planned or gripper control
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

#If a step fails, rx200 attempts to return to home position and publishes failure
    def fail_sequence(self, reason):

        self.get_logger().error(reason)
#if the return fails, rx200 stops
        if self.returning_home:
            self.get_logger().error('Return home failed')
            return

        self.returning_home = True

#Prevents a return home being published as a success 
        self.publish_status('FAILED')

        self.sequence_steps = [
            ('arm', self.home_pose, 'return home after failure')
        ]

        self.step_index = 0
        self.busy = True

        self.send_next_step()
        
#Creating the end-effector pose
    def send_ee_pose(self, x, y, z, w=1.0, cartesian=False):
    
        p = PoseStamped()
        p.header.frame_id = self.ref_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = float(z)
        yaw = math.atan2(y, x)
        roll = 0.0
        pitch = 0.0

        cos_roll = math.cos(roll / 2)
        sine_roll =  math.sin(roll / 2)
        cos_pitch = math.cos(pitch / 2)
        sine_pitch = math.sin(pitch / 2)
        cos_yaw = math.cos(yaw / 2)
        sine_yaw = math.sin(yaw / 2)
        p.pose.orientation = Quaternion(
            x=sine_roll * cos_pitch * cos_yaw - cos_roll * sine_pitch * sine_yaw,
            y=cos_roll * sine_pitch * cos_yaw + sine_roll * cos_pitch * sine_yaw,
            z=cos_roll * cos_pitch * sine_yaw - sine_roll * sine_pitch * cos_yaw,
            w=cos_roll * cos_pitch * cos_yaw + sine_roll * sine_pitch * sine_yaw,
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
 
 #Storing measured joint-state messages for Cartesian planning   
    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

#Using Cartesian path planning to create straight-line movement
    def plan_cartesian_path(self, pose_stamped):

#Get the actual current end-effector position
        try:
            transform = self.tf_buffer.lookup_transform(
                self.ref_frame, self.ee_link, Time()
            )
        except Exception as exc:
            self.fail_sequence(f'Cannot read current EE transform: {exc}')
            return

        current = transform.transform

#Check how far the actual position is from the requested position
        x_error = pose_stamped.pose.position.x - current.translation.x
        y_error = pose_stamped.pose.position.y - current.translation.y
        xy_error = math.hypot(x_error, y_error)

        self.get_logger().info(
            f'Cartesian start EE: ({current.translation.x:.4f}, 'f'{current.translation.y:.4f}, {current.translation.z:.4f}); 'f'XY offset from nominal target: {xy_error * 1000:.1f} mm'
        )

#Reject large positioning errors for Cartesian Movement
        if self.step_index in (4, 8) and xy_error > 0.010:
            self.fail_sequence(
                f'Actual EE is {xy_error * 1000:.1f} mm from requested X/Y; '
                'aborting Cartesian movement'
            )
            return

#Keep actual X, Y and orientation for vertical movement
        pose_stamped.pose.position.x = current.translation.x
        pose_stamped.pose.position.y = current.translation.y
        pose_stamped.pose.orientation = current.rotation

#Checking that measured joint positions are available
        if self.latest_joint_state is None:
            self.fail_sequence('No joint states received')
            return

        # Get the actual positions of the five arm joints
        joint_positions = dict(zip(
            self.latest_joint_state.name,
            self.latest_joint_state.position
        ))

        arm_joints = [
            'waist',
            'shoulder',
            'elbow',
            'wrist_angle',
            'wrist_rotate'
        ]
#Creating a  Cartesian path request
        request = GetCartesianPath.Request()

        request.header = pose_stamped.header

#Using actual measured joint positions as the starting state
#Previous attempts failed and on inspection, expected position differered from actual
        request.start_state.is_diff = True
        request.start_state.joint_state.name = arm_joints
        request.start_state.joint_state.position = [
            float(joint_positions[joint]) for joint in arm_joints
        ]

        request.group_name = self.arm_group
        request.link_name = self.ee_link
        request.waypoints = [pose_stamped.pose]

        request.max_step = 0.001
        request.jump_threshold = 1.5
        request.avoid_collisions = True

        request.max_velocity_scaling_factor = 0.10
        request.max_acceleration_scaling_factor = 0.10

#Requesting the Cartesian trajectory
        future = self._cartesian_client.call_async(request)
        future.add_done_callback(self._cartesian_path_cb)

#Detailing the planning and rejecting an incomplete Cartesian path plan
    def _cartesian_path_cb(self, future):
        try:
            result = future.result()
            self.get_logger().info(
                f'Cartesian path fraction: {result.fraction:.3f}'
            )
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                if self.allow_cartesian_fallback:
                    kind, value, label = self.sequence_steps[self.step_index]
                    self.get_logger().warning(
                        f'Using standard MoveIt planning for: {label}'
                    )
                    self.send_ee_pose(*value)
                else:
                    self.fail_sequence(
                        f'Cartesian planning failed: code {result.error_code.val}'
                    )
                return

            # Never execute a partial approach or lift.
            if result.fraction < 0.999 or not result.solution.joint_trajectory.points:

                self.get_logger().warning(
                    f'Incomplete Cartesian path ({result.fraction * 100:.1f}%)'
                )
#If the Cartesian planning fails, normal MoveIt is used to allow completion of task
                if self.allow_cartesian_fallback:

                    kind, value, label = self.sequence_steps[self.step_index]

                    self.get_logger().warning(
                        f'Using standard MoveIt planning for: {label}'
                    )

                    self.send_ee_pose(*value)

                else:
                    self.fail_sequence('Cartesian path incomplete - fallback disabled')

                return

            if not self._execute_client.server_is_ready():
                self.fail_sequence('ExecuteTrajectory action unavailable')
                return

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = result.solution
            execute_future = self._execute_client.send_goal_async(goal)
            execute_future.add_done_callback(self._goal_response_cb)
        except Exception as exc:
            self.fail_sequence(f'Cartesian path error: {exc}')


    def send_gr_pose(self, open=True, position = None):
        req = MotionPlanRequest()
        req.group_name = self.gripper_group
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
#Measured Gripper values to grip the object safely
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

#Resuming execution after movement has settled
    def resume_after_settle(self):
        self.settle_timer.cancel()
        self.destroy_timer(self.settle_timer)
        self.settle_timer = None

        if self.busy:
            self.send_next_step()

    def _result_cb(self, future):
        try:
            wrapped_result = future.result()
            code = wrapped_result.result.error_code.val
            self.get_logger().info(f'[RESULTS] Result Code: {code}')
            if code != MoveItErrorCodes.SUCCESS:
                self.fail_sequence(f'MoveIt failed at step {self.step_index + 1}: code {code}')
                return
            self.step_index += 1
            self.settle_timer = self.create_timer(
                0.75 , self.resume_after_settle
            )
            
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
