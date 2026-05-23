import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RgbToBgrNode(Node):
    def __init__(self):
        super().__init__('rgb_to_bgr_node')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_bgr')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, output_topic, 10)
        self._sub = self.create_subscription(Image, input_topic, self._on_image, 10)

    def _on_image(self, msg: Image) -> None:
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}')
            return

        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        out_msg = self._bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
        out_msg.header = msg.header
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
