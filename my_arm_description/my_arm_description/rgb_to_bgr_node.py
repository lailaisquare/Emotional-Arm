import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class RgbToBgrNode(Node):
    def __init__(self):
        super().__init__('rgb_to_bgr_node')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_bgr')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(Image, output_topic, 10)
        self._sub = self.create_subscription(Image, input_topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}')
            return

        if msg.encoding == 'bgr8':
            self._pub.publish(msg)
            return

        data = bytearray(msg.data)
        for i in range(0, len(data), 3):
            data[i], data[i + 2] = data[i + 2], data[i]

        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height = msg.height
        out_msg.width = msg.width
        out_msg.encoding = 'bgr8'
        out_msg.is_bigendian = msg.is_bigendian
        out_msg.step = msg.step
        out_msg.data = bytes(data)
        self._pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RgbToBgrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
