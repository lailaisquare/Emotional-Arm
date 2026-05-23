#!/usr/bin/env python3
"""Face tracking visual servo for Feetech ST3215 servos."""

import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import serial


HEADER = b"\xFF\xFF"
INST_WRITE = 0x03


def checksum(data: bytes) -> int:
    return (~sum(data) & 0xFF)


class FeetechBus:
    def __init__(self, port: str, baudrate: int, timeout: float):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=timeout,
        )
        self._lock = threading.Lock()

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def _build_packet(self, servo_id: int, instruction: int, params: bytes) -> bytes:
        length = len(params) + 2
        body = bytes([servo_id, length, instruction]) + params
        return HEADER + body + bytes([checksum(body)])

    def _read_exact(self, size: int) -> Optional[bytes]:
        data = self.ser.read(size)
        if len(data) != size:
            return None
        return data

    def write_register(self, servo_id: int, addr: int, data: bytes, wait_status: bool = True) -> bool:
        packet = self._build_packet(servo_id, INST_WRITE, bytes([addr]) + data)
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write(packet)
            self.ser.flush()

            if not wait_status:
                return True

            header = self._read_exact(2)
            if header != HEADER:
                return False

            head_rest = self._read_exact(2)
            if head_rest is None:
                return False

            resp_id, length = head_rest[0], head_rest[1]
            if resp_id != servo_id:
                return False

            payload = self._read_exact(length)
            if payload is None:
                return False

            chk_expected = checksum(bytes([resp_id, length]) + payload[:-1])
            chk_recv = payload[-1]
            if chk_expected != chk_recv:
                return False

            err = payload[0]
            return err == 0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def degree_to_raw(degree: float, raw_max: int) -> int:
    degree = degree % 360.0
    raw = int(round((degree / 360.0) * raw_max))
    return max(0, min(raw_max, raw))


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


class FaceServoTracker(Node):
    def __init__(self) -> None:
        super().__init__('face_servo_tracker')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_encoding', 'bgr8')
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 4)
        self.declare_parameter('min_face_size', 60)

        self.declare_parameter('servo_port', '/dev/ttyUSB0')
        self.declare_parameter('servo_baudrate', 1000000)
        self.declare_parameter('servo_timeout', 0.05)
        self.declare_parameter('servo_wait_status', True)
        self.declare_parameter('servo_endian', 'little')
        self.declare_parameter('servo_raw_max', 4095)
        self.declare_parameter('servo_pos_addr', 42)

        self.declare_parameter('pan_id', 1)
        self.declare_parameter('tilt_id', 2)
        self.declare_parameter('pan_center_deg', 180.0)
        self.declare_parameter('tilt_center_deg', 180.0)
        self.declare_parameter('pan_min_deg', 0.0)
        self.declare_parameter('pan_max_deg', 360.0)
        self.declare_parameter('tilt_min_deg', 0.0)
        self.declare_parameter('tilt_max_deg', 360.0)

        self.declare_parameter('gain_pan', 5.0)
        self.declare_parameter('gain_tilt', 5.0)
        self.declare_parameter('step_limit_deg', 3.0)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('return_to_center', False)
        self.declare_parameter('lost_timeout', 1.0)

        self._image_encoding = self.get_parameter('image_encoding').get_parameter_value().string_value
        self._scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value
        self._min_neighbors = self.get_parameter('min_neighbors').get_parameter_value().integer_value
        self._min_face_size = self.get_parameter('min_face_size').get_parameter_value().integer_value

        self._pan_id = self.get_parameter('pan_id').get_parameter_value().integer_value
        self._tilt_id = self.get_parameter('tilt_id').get_parameter_value().integer_value
        self._pan_deg = self.get_parameter('pan_center_deg').get_parameter_value().double_value
        self._tilt_deg = self.get_parameter('tilt_center_deg').get_parameter_value().double_value
        self._pan_min = self.get_parameter('pan_min_deg').get_parameter_value().double_value
        self._pan_max = self.get_parameter('pan_max_deg').get_parameter_value().double_value
        self._tilt_min = self.get_parameter('tilt_min_deg').get_parameter_value().double_value
        self._tilt_max = self.get_parameter('tilt_max_deg').get_parameter_value().double_value

        self._gain_pan = self.get_parameter('gain_pan').get_parameter_value().double_value
        self._gain_tilt = self.get_parameter('gain_tilt').get_parameter_value().double_value
        self._step_limit = self.get_parameter('step_limit_deg').get_parameter_value().double_value
        self._update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self._return_to_center = self.get_parameter('return_to_center').get_parameter_value().bool_value
        self._lost_timeout = self.get_parameter('lost_timeout').get_parameter_value().double_value

        self._raw_max = self.get_parameter('servo_raw_max').get_parameter_value().integer_value
        self._pos_addr = self.get_parameter('servo_pos_addr').get_parameter_value().integer_value
        self._wait_status = self.get_parameter('servo_wait_status').get_parameter_value().bool_value

        port = self.get_parameter('servo_port').get_parameter_value().string_value
        baud = self.get_parameter('servo_baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('servo_timeout').get_parameter_value().double_value
        self._endian = self.get_parameter('servo_endian').get_parameter_value().string_value

        try:
            self._bus = FeetechBus(port, baud, timeout)
        except Exception as exc:
            self.get_logger().error(f'failed to open servo port: {exc}')
            self._bus = None

        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self._cascade = cv2.CascadeClassifier(cascade_path)
        if self._cascade.empty():
            self.get_logger().error(f'failed to load cascade: {cascade_path}')

        self._last_cmd = 0.0
        self._last_face = 0.0

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._sub = self.create_subscription(Image, image_topic, self._on_image, 10)

    def destroy_node(self):
        if self._bus is not None:
            self._bus.close()
        return super().destroy_node()

    def _send_angle(self, servo_id: int, degree: float) -> None:
        if self._bus is None:
            return
        raw = degree_to_raw(degree, self._raw_max)
        payload = raw.to_bytes(2, byteorder=self._endian, signed=False)
        ok = self._bus.write_register(servo_id, self._pos_addr, payload, wait_status=self._wait_status)
        if not ok:
            self.get_logger().warn(f'write failed for servo {servo_id}')

    def _update_targets(self, dx: float, dy: float) -> None:
        step_pan = clamp(self._gain_pan * dx, -self._step_limit, self._step_limit)
        step_tilt = clamp(self._gain_tilt * dy, -self._step_limit, self._step_limit)

        self._pan_deg = clamp(self._pan_deg + step_pan, self._pan_min, self._pan_max)
        self._tilt_deg = clamp(self._tilt_deg + step_tilt, self._tilt_min, self._tilt_max)

        self._send_angle(self._pan_id, self._pan_deg)
        self._send_angle(self._tilt_id, self._tilt_deg)

    def _return_center(self) -> None:
        self._pan_deg = clamp(self._pan_deg, self._pan_min, self._pan_max)
        self._tilt_deg = clamp(self._tilt_deg, self._tilt_min, self._tilt_max)
        self._send_angle(self._pan_id, self._pan_deg)
        self._send_angle(self._tilt_id, self._tilt_deg)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self._update_rate > 0:
            min_dt = 1.0 / self._update_rate
            if now - self._last_cmd < min_dt:
                return

        img = decode_image(msg, self._image_encoding)
        if img is None:
            return

        if self._image_encoding == 'rgb8':
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        elif self._image_encoding == 'bgr8':
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img

        min_size = (self._min_face_size, self._min_face_size)
        faces = self._cascade.detectMultiScale(
            gray,
            scaleFactor=self._scale_factor,
            minNeighbors=int(self._min_neighbors),
            minSize=min_size,
        )

        if len(faces) == 0:
            if self._return_to_center and (now - self._last_face) > self._lost_timeout:
                self._pan_deg = self.get_parameter('pan_center_deg').get_parameter_value().double_value
                self._tilt_deg = self.get_parameter('tilt_center_deg').get_parameter_value().double_value
                self._return_center()
                self._last_cmd = now
            return

        self._last_face = now

        x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
        cx = x + w / 2.0
        cy = y + h / 2.0

        img_cx = msg.width / 2.0
        img_cy = msg.height / 2.0

        dx = (img_cx - cx) / img_cx
        dy = (img_cy - cy) / img_cy

        self._update_targets(dx, dy)
        self._last_cmd = now


def main(args=None) -> int:
    rclpy.init(args=args)
    node = FaceServoTracker()
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
