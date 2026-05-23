#!/usr/bin/env python3
"""Face tracking using MediaPipe and MoveIt2 joint limits/collision constraints."""

import time
from pathlib import Path
from typing import List, Optional, Sequence

import cv2
import numpy as np
try:
    from mediapipe.tasks import python as mp_tasks
    from mediapipe.tasks.python import vision
    from mediapipe.tasks.python.vision.core import image as mp_image
    _MP_AVAILABLE = True
except Exception:  # noqa: BLE001
    mp_tasks = None
    vision = None
    mp_image = None
    _MP_AVAILABLE = False
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from sensor_msgs.msg import Image
from shape_msgs.msg import SolidPrimitive
from pymoveit2 import MoveIt2, MoveIt2State


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def decode_image(msg: Image, encoding: str) -> Optional[np.ndarray]:
    if encoding not in ('rgb8', 'bgr8', 'mono8'):
        return None

    if encoding == 'mono8':
        if msg.step < msg.width:
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if len(buf) < msg.step * msg.height:
            return None
        img = buf.reshape((msg.height, msg.step))
        return img[:, :msg.width]

    channels = 3
    if msg.step < msg.width * channels:
        return None
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if len(buf) < msg.step * msg.height:
        return None
    img = buf.reshape((msg.height, msg.step))
    img = img[:, :msg.width * channels]
    return img.reshape((msg.height, msg.width, channels))


def parse_joint_names(value) -> List[str]:
    if isinstance(value, (list, tuple)):
        return [str(v) for v in value]
    if isinstance(value, str):
        return [v.strip() for v in value.split(',') if v.strip()]
    return []


class FaceMoveItTracker(Node):
    def __init__(self) -> None:
        super().__init__('face_moveit_tracker')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_encoding', 'bgr8')
        self.declare_parameter('min_detection_confidence', 0.6)
        self.declare_parameter('model_selection', 0)
        self.declare_parameter('model_path', '')

        self.declare_parameter('joint_names', 'l1,l2,l3,l4,l5,l6')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector', 'lamp_link_1')
        self.declare_parameter('group_name', 'arm_group')
        self.declare_parameter('use_move_group_action', True)
        self.declare_parameter('ignore_new_calls', True)
        self.declare_parameter('max_velocity', 0.3)
        self.declare_parameter('max_acceleration', 0.3)

        self.declare_parameter('pan_joint', 'l1')
        self.declare_parameter('tilt_joint', 'l2')
        self.declare_parameter('pan_center', 0.0)
        self.declare_parameter('tilt_center', 0.0)
        self.declare_parameter('pan_min', -1.57)
        self.declare_parameter('pan_max', 1.57)
        self.declare_parameter('tilt_min', -1.0)
        self.declare_parameter('tilt_max', 1.0)

        self.declare_parameter('gain_pan', 0.6)
        self.declare_parameter('gain_tilt', 0.6)
        self.declare_parameter('max_step', 0.2)
        self.declare_parameter('update_rate', 3.0)
        self.declare_parameter('return_to_center', False)
        self.declare_parameter('lost_timeout', 1.0)

        self.declare_parameter('add_table', True)
        self.declare_parameter('table_id', 'table')
        self.declare_parameter('table_frame', 'base_link')
        self.declare_parameter('table_size_x', 1.0)
        self.declare_parameter('table_size_y', 1.0)
        self.declare_parameter('table_size_z', 0.05)
        self.declare_parameter('table_center_x', 0.0)
        self.declare_parameter('table_center_y', 0.0)
        self.declare_parameter('table_center_z', -0.2)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._encoding = self.get_parameter('image_encoding').get_parameter_value().string_value

        joint_names = parse_joint_names(
            self.get_parameter('joint_names').get_parameter_value().string_value
        )
        if not joint_names:
            joint_names = ['l1', 'l2', 'l3', 'l4', 'l5', 'l6']

        base_link = self.get_parameter('base_link').get_parameter_value().string_value
        end_effector = self.get_parameter('end_effector').get_parameter_value().string_value
        group_name = self.get_parameter('group_name').get_parameter_value().string_value

        use_move_group_action = self.get_parameter('use_move_group_action').get_parameter_value().bool_value
        ignore_new_calls = self.get_parameter('ignore_new_calls').get_parameter_value().bool_value

        self._moveit = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link,
            end_effector_name=end_effector,
            group_name=group_name,
            use_move_group_action=use_move_group_action,
            ignore_new_calls_while_executing=ignore_new_calls,
        )

        self._moveit.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self._moveit.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value

        self._joint_names = joint_names
        self._joint_index = {name: idx for idx, name in enumerate(joint_names)}

        self._pan_joint = self.get_parameter('pan_joint').get_parameter_value().string_value
        self._tilt_joint = self.get_parameter('tilt_joint').get_parameter_value().string_value
        if self._pan_joint not in self._joint_index or self._tilt_joint not in self._joint_index:
            self.get_logger().error('pan_joint or tilt_joint not in joint_names')

        self._pan_center = self.get_parameter('pan_center').get_parameter_value().double_value
        self._tilt_center = self.get_parameter('tilt_center').get_parameter_value().double_value
        self._pan_min = self.get_parameter('pan_min').get_parameter_value().double_value
        self._pan_max = self.get_parameter('pan_max').get_parameter_value().double_value
        self._tilt_min = self.get_parameter('tilt_min').get_parameter_value().double_value
        self._tilt_max = self.get_parameter('tilt_max').get_parameter_value().double_value

        self._gain_pan = self.get_parameter('gain_pan').get_parameter_value().double_value
        self._gain_tilt = self.get_parameter('gain_tilt').get_parameter_value().double_value
        self._max_step = self.get_parameter('max_step').get_parameter_value().double_value
        self._update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self._return_to_center = self.get_parameter('return_to_center').get_parameter_value().bool_value
        self._lost_timeout = self.get_parameter('lost_timeout').get_parameter_value().double_value

        self._last_cmd = 0.0
        self._last_face = 0.0
        self._last_positions: Optional[List[float]] = None

        conf = self.get_parameter('min_detection_confidence').get_parameter_value().double_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            model_path = str(Path.home() / '.cache' / 'mediapipe' / 'face_detector_short_range.tflite')
        model_file = Path(model_path).expanduser()

        if not _MP_AVAILABLE:
            self.get_logger().error(
                'MediaPipe Tasks API not available. Install with: pip3 install mediapipe. '
                'Also check for a local mediapipe.py shadowing the package.'
            )
            raise RuntimeError('mediapipe tasks not available')

        if not model_file.exists():
            self.get_logger().error(
                f'MediaPipe model not found: {model_file}. Download the face detector tflite model '
                'and pass --ros-args -p model_path:=/path/to/model.tflite'
            )
            raise RuntimeError('mediapipe model missing')

        options = vision.FaceDetectorOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=str(model_file)),
            min_detection_confidence=float(conf),
        )
        self._detector = vision.FaceDetector.create_from_options(options)

        self._sub = self.create_subscription(Image, image_topic, self._on_image, 10)

        self._collision_pub = None
        self._table_timer = None
        self._table_publish_count = 0
        self._table_publish_limit = 5

        if self.get_parameter('add_table').get_parameter_value().bool_value:
            self._collision_pub = self.create_publisher(
                CollisionObject,
                '/collision_object',
                QoSProfile(
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
            )
            self._table_timer = self.create_timer(0.5, self._publish_table)

    def _publish_table(self) -> None:
        if self._collision_pub is None:
            return

        table = CollisionObject()
        table.header.frame_id = self.get_parameter('table_frame').get_parameter_value().string_value
        table.id = self.get_parameter('table_id').get_parameter_value().string_value

        size_x = self.get_parameter('table_size_x').get_parameter_value().double_value
        size_y = self.get_parameter('table_size_y').get_parameter_value().double_value
        size_z = self.get_parameter('table_size_z').get_parameter_value().double_value

        center_x = self.get_parameter('table_center_x').get_parameter_value().double_value
        center_y = self.get_parameter('table_center_y').get_parameter_value().double_value
        center_z = self.get_parameter('table_center_z').get_parameter_value().double_value

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size_x), float(size_y), float(size_z)]

        pose = Pose()
        pose.position.x = float(center_x)
        pose.position.y = float(center_y)
        pose.position.z = float(center_z)
        pose.orientation.w = 1.0

        table.primitives = [primitive]
        table.primitive_poses = [pose]
        table.operation = CollisionObject.ADD

        self._collision_pub.publish(table)
        self._table_publish_count += 1
        if self._table_timer is not None and self._table_publish_count >= self._table_publish_limit:
            self._table_timer.cancel()

    def _current_positions(self) -> Optional[List[float]]:
        joint_state = self._moveit.joint_state
        if joint_state is None or not joint_state.name:
            return self._last_positions

        name_to_pos = {name: pos for name, pos in zip(joint_state.name, joint_state.position)}
        positions = []
        for name in self._joint_names:
            if name in name_to_pos:
                positions.append(float(name_to_pos[name]))
            else:
                return self._last_positions
        self._last_positions = positions
        return positions

    def _send_moveit(self, positions: Sequence[float]) -> None:
        if self._moveit.query_state() != MoveIt2State.IDLE:
            return
        self._moveit.move_to_configuration(list(positions), joint_names=self._joint_names)

    def _return_center(self) -> None:
        positions = self._current_positions()
        if positions is None:
            return
        positions = list(positions)
        if self._pan_joint in self._joint_index:
            positions[self._joint_index[self._pan_joint]] = clamp(self._pan_center, self._pan_min, self._pan_max)
        if self._tilt_joint in self._joint_index:
            positions[self._joint_index[self._tilt_joint]] = clamp(self._tilt_center, self._tilt_min, self._tilt_max)
        self._send_moveit(positions)

    def _update_targets(self, dx: float, dy: float) -> None:
        positions = self._current_positions()
        if positions is None:
            return

        positions = list(positions)
        step_pan = clamp(self._gain_pan * dx, -self._max_step, self._max_step)
        step_tilt = clamp(self._gain_tilt * dy, -self._max_step, self._max_step)

        if self._pan_joint in self._joint_index:
            idx = self._joint_index[self._pan_joint]
            positions[idx] = clamp(positions[idx] + step_pan, self._pan_min, self._pan_max)
        if self._tilt_joint in self._joint_index:
            idx = self._joint_index[self._tilt_joint]
            positions[idx] = clamp(positions[idx] + step_tilt, self._tilt_min, self._tilt_max)

        self._send_moveit(positions)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self._update_rate > 0:
            min_dt = 1.0 / self._update_rate
            if now - self._last_cmd < min_dt:
                return

        img = decode_image(msg, self._encoding)
        if img is None:
            return

        if self._encoding == 'rgb8':
            rgb = img
        elif self._encoding == 'bgr8':
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        rgb = np.ascontiguousarray(rgb)
        mp_img = mp_image.Image(image_format=mp_image.ImageFormat.SRGB, data=rgb)
        result = self._detector.detect(mp_img)

        if not result.detections:
            if self._return_to_center and (now - self._last_face) > self._lost_timeout:
                self._return_center()
                self._last_cmd = now
            return

        best = max(
            result.detections,
            key=lambda d: (d.categories[0].score if d.categories else 0.0),
        )
        bbox = best.bounding_box
        cx = bbox.origin_x + bbox.width / 2.0
        cy = bbox.origin_y + bbox.height / 2.0

        img_cx = msg.width / 2.0
        img_cy = msg.height / 2.0
        dx = (img_cx - cx) / img_cx
        dy = (img_cy - cy) / img_cy

        self._last_face = now
        self._update_targets(dx, dy)
        self._last_cmd = now


def main(args=None) -> int:
    rclpy.init(args=args)
    node = FaceMoveItTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
