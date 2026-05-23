import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class ArucoKeepaliveNode(Node):
    def __init__(self) -> None:
        super().__init__('aruco_keepalive_node')
        self.declare_parameter('pose_topic', '/aruco_single/pose')
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self._sub = self.create_subscription(PoseStamped, pose_topic, self._on_pose, 10)

    def _on_pose(self, _msg: PoseStamped) -> None:
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoKeepaliveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
