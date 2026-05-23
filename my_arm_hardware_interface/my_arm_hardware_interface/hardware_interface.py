import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from arm_hardware_interface.motors.feetech import FeetechMotorsBus
from serial.serialutil import SerialException


class HardwareInterface(Node):
    def __init__(self):
        super().__init__('my_arm_hardware_interface')

        self.declare_parameter('port', '/dev/ttyARM0')
        self.declare_parameter('publish_topic', 'joint_states')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter(
            'action_name',
            '/arm_controller/follow_joint_trajectory'
        )
        self.declare_parameter('trajectory_rate', 100.0)
        self.declare_parameter('motor_model', 'sts3215')
        self.declare_parameter('joint_names', ['l1', 'l2', 'l3', 'l4', 'l5', 'l6'])
        self.declare_parameter('motor_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('use_trajectory_speed', True)
        self.declare_parameter('default_goal_speed', 100)
        self.declare_parameter('default_acceleration', 50)
        self.declare_parameter('min_goal_speed', 20)
        self.declare_parameter('max_goal_speed', 1000)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self.trajectory_rate = self.get_parameter('trajectory_rate').get_parameter_value().double_value
        self.motor_model = self.get_parameter('motor_model').get_parameter_value().string_value
        self.joint_names = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)
        self.motor_ids = list(self.get_parameter('motor_ids').get_parameter_value().integer_array_value)
        self.use_trajectory_speed = self.get_parameter('use_trajectory_speed').get_parameter_value().bool_value
        self.default_goal_speed = int(
            self.get_parameter('default_goal_speed').get_parameter_value().integer_value
        )
        self.default_acceleration = int(
            self.get_parameter('default_acceleration').get_parameter_value().integer_value
        )
        self.min_goal_speed = int(
            self.get_parameter('min_goal_speed').get_parameter_value().integer_value
        )
        self.max_goal_speed = int(
            self.get_parameter('max_goal_speed').get_parameter_value().integer_value
        )

        if len(self.joint_names) != len(self.motor_ids):
            raise ValueError('joint_names and motor_ids must be the same length')

        self.callback_group = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(JointState, self.publish_topic, 10)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=self.callback_group,
        )

        publish_rate = max(self.publish_rate, 1.0)
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_real_joint_states,
            callback_group=self.callback_group,
        )

        trajectory_rate = max(self.trajectory_rate, 1.0)
        self.command_timer = self.create_timer(
            1.0 / trajectory_rate,
            self.execute_trajectory_step,
            callback_group=self.callback_group,
        )

        self.motors_bus = FeetechMotorsBus(
            port=self.port,
            motors={},
        )
        self.motors_bus.connect()
        self.bus_lock = threading.Lock()

        self.joint_to_motor_id = dict(zip(self.joint_names, self.motor_ids))
        self.motor_names = list(self.joint_to_motor_id.keys())
        self.motors_bus.motors = {
            name: (self.joint_to_motor_id[name], self.motor_model)
            for name in self.motor_names
        }

        self.configure_motors()
        self.set_torque(True)

        self.trajectory_points = []
        self.trajectory_start = None
        self.trajectory_index = 0
        self.trajectory_active = False
        self.trajectory_cancel_requested = False
        self.trajectory_done_event = threading.Event()
        self.active_goal_handle = None
        self.speed_segment_index = -1

    def configure_motors(self):
        with self.bus_lock:
            for name in self.motor_names:
                self.motors_bus.write('Goal_Speed', self.default_goal_speed, name)
                self.motors_bus.write('Acceleration', self.default_acceleration, name)

    def set_torque(self, enable):
        torque_value = 1 if enable else 0
        with self.bus_lock:
            for name in self.motor_names:
                self.motors_bus.write('Torque_Enable', torque_value, name)
        self.get_logger().info(
            f"Torque {'enabled' if enable else 'disabled'} for all motors."
        )

    def goal_callback(self, goal_request):
        trajectory = goal_request.trajectory
        if not trajectory.points:
            return GoalResponse.REJECT

        name_to_index = {name: idx for idx, name in enumerate(trajectory.joint_names)}
        missing = [name for name in self.motor_names if name not in name_to_index]
        if missing:
            self.get_logger().error(
                f"Trajectory missing joints: {missing}. Rejecting goal."
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.trajectory_cancel_requested = True
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        if self.active_goal_handle and self.active_goal_handle.is_active:
            self.active_goal_handle.abort()
        self.active_goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        result = FollowJointTrajectory.Result()
        points = self.build_trajectory_points(goal_handle.request.trajectory)
        if not points:
            result.error_code = -1
            result.error_string = 'Empty trajectory.'
            goal_handle.abort()
            return result

        self.start_trajectory(points)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.trajectory_cancel_requested = True
            if self.trajectory_done_event.wait(timeout=0.05):
                break

        if self.trajectory_cancel_requested:
            result.error_code = -1
            result.error_string = 'Trajectory canceled.'
            goal_handle.canceled()
            return result

        result.error_code = 0
        goal_handle.succeed()
        return result

    def build_trajectory_points(self, trajectory):
        if not trajectory.points:
            return []

        name_to_index = {name: idx for idx, name in enumerate(trajectory.joint_names)}
        points = []
        for point in trajectory.points:
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            positions = [point.positions[name_to_index[name]] for name in self.motor_names]
            points.append((t, positions))
        return points

    def start_trajectory(self, points):
        self.trajectory_points = points
        self.trajectory_start = self.get_clock().now()
        self.trajectory_index = 0
        self.speed_segment_index = -1
        self.trajectory_active = True
        self.trajectory_cancel_requested = False
        self.trajectory_done_event.clear()

    def stop_trajectory(self):
        self.trajectory_active = False
        self.trajectory_points = []
        self.trajectory_done_event.set()

    def execute_trajectory_step(self):
        if not self.trajectory_active or not self.trajectory_points:
            return

        if self.trajectory_cancel_requested:
            self.stop_trajectory()
            return

        now = self.get_clock().now()
        elapsed = (now - self.trajectory_start).nanoseconds * 1e-9
        points = self.trajectory_points

        if elapsed <= points[0][0]:
            target_positions = points[0][1]
        elif elapsed >= points[-1][0]:
            target_positions = points[-1][1]
            self.trajectory_active = False
            self.trajectory_done_event.set()
        else:
            idx = self.trajectory_index
            while idx + 1 < len(points) and points[idx + 1][0] <= elapsed:
                idx += 1
            t0, p0 = points[idx]
            t1, p1 = points[idx + 1]
            if self.use_trajectory_speed and idx != self.speed_segment_index:
                self.update_segment_speed(p0, p1, t1 - t0)
                self.speed_segment_index = idx
            ratio = (elapsed - t0) / (t1 - t0) if t1 > t0 else 1.0
            target_positions = [
                p0[i] + (p1[i] - p0[i]) * ratio
                for i in range(len(p0))
            ]
            self.trajectory_index = idx

        self.send_joint_positions(target_positions)

    def send_joint_positions(self, positions):
        motor_ids = []
        motor_values = []
        motor_models = []

        for name, position in zip(self.motor_names, positions):
            motor_id = self.joint_to_motor_id[name]
            motor_value = int((-position + math.pi) * (4095 / (2 * math.pi)))
            motor_ids.append(motor_id)
            motor_values.append(motor_value)
            motor_models.append(self.motor_model)

        if motor_ids:
            try:
                with self.bus_lock:
                    self.motors_bus.write_with_motor_ids(
                        motor_models,
                        motor_ids,
                        'Goal_Position',
                        motor_values
                    )
            except (ConnectionError, SerialException) as exc:
                self.get_logger().error(f"Write failed: {exc}")
                if self.trajectory_active:
                    self.trajectory_cancel_requested = True
                    self.stop_trajectory()

    def update_segment_speed(self, start_positions, end_positions, dt):
        if dt <= 0.0:
            return

        rad_to_steps = 4095.0 / (2 * math.pi)
        with self.bus_lock:
            for name, start, end in zip(self.motor_names, start_positions, end_positions):
                delta_steps = abs(end - start) * rad_to_steps
                speed = int(delta_steps / dt)
                speed = max(self.min_goal_speed, min(speed, self.max_goal_speed))
                self.motors_bus.write('Goal_Speed', speed, name)
                self.motors_bus.write('Acceleration', self.default_acceleration, name)

    def publish_real_joint_states(self):
        motor_ids = list(self.joint_to_motor_id.values())
        motor_models = [self.motor_model] * len(motor_ids)

        try:
            with self.bus_lock:
                positions = self.motors_bus.read_with_motor_ids(
                    motor_models,
                    motor_ids,
                    'Present_Position'
                )
        except (ConnectionError, SerialException) as exc:
            self.get_logger().error(f"Connection error: {exc}")
            self.get_logger().info('Attempting to reconnect...')
            while True:
                try:
                    with self.bus_lock:
                        self.motors_bus.reconnect()
                    self.get_logger().info('Reconnected to motors.')
                    return
                except Exception as retry_exc:
                    self.get_logger().error(f"Reconnection failed: {retry_exc}")
                    time.sleep(1)

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.motor_names
        joint_state_msg.position = [
            -((pos / 4095.0) * (2 * math.pi) - math.pi)
            for pos in positions
        ]

        self.publisher_.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    hardware_interface = HardwareInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(hardware_interface)
    try:
        executor.spin()
    finally:
        hardware_interface.motors_bus.disconnect()
        hardware_interface.destroy_node()
        executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

