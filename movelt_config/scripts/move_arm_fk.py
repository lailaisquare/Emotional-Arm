#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
正运动学 (FK)：给定关节角度，查询末端执行器 lamp_link_1 相对 base_link 的位姿
启动时自动归零初始化 TF 树
"""

import time
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import tf2_ros
from rclpy.time import Time


def main():
    rclpy.init()
    node = Node('arm_fk_controller')

    joint_names = ['l1', 'l2', 'l3', 'l4', 'l5', 'l6']

    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name='base_link',
        end_effector_name='lamp_link_1',
        group_name='arm_group',
    )
    moveit2.max_velocity = 0.3
    moveit2.max_acceleration = 0.3
    time.sleep(1.0)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    node.get_logger().info("✅ 正运动学查询器已就绪")

    # --- 关键：自动初始化，让 robot_state_publisher 建立 TF ---
    node.get_logger().info("🔄 正在初始化 (归零) 以建立 TF 树...")
    home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    moveit2.move_to_configuration(home_positions)
    moveit2.wait_until_executed()
    time.sleep(1.0)  # 等待 TF 完全建立
    node.get_logger().info("✅ TF 树已就绪!")
    # -----------------------------------------------------------

    print("\n" + "=" * 50)
    print("  📐 正运动学 (FK)")
    print("    查询末端 lamp_link_1 的位姿")
    print("=" * 50)
    print("  c   - 查询当前末端位姿")
    print("  0   - 回到 home 并查询")
    print("  1   - 姿态1 并查询")
    print("  2   - 姿态2 并查询")
    print("  自定义 - 输入6个角度 (逗号分隔)")
    print("  q   - 退出")
    print("=" * 50 + "\n")

    presets = {
        '0': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        '1': [0.0, -0.3, 0.5, 0.0, 0.3, 0.0],
        '2': [0.0, -0.8, 1.2, 0.0, 0.6, 0.0],
    }

    def query_fk():
        """查询 base_link → lamp_link_1 的变换"""
        try:
            transform = tf_buffer.lookup_transform(
                'base_link',
                'lamp_link_1',
                Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            pos = transform.transform.translation
            rot = transform.transform.rotation
            print(f"\n📍 末端 lamp_link_1 (相对 base_link):")
            print(f"   位置: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")
            print(f"   姿态: x={rot.x:.4f}, y={rot.y:.4f}, z={rot.z:.4f}, w={rot.w:.4f}")
            print()
            return True
        except Exception as e:
            node.get_logger().error(f"❌ TF 查询失败: {e}")
            return False

    # 启动后立即显示当前位姿
    query_fk()

    try:
        while rclpy.ok():
            choice = input("请输入选项: ").strip().lower()

            if choice == 'q':
                break

            elif choice == 'c':
                query_fk()

            elif choice in presets:
                positions = presets[choice]
                node.get_logger().info(f"移动到: {[f'{p:.2f}' for p in positions]}")
                moveit2.move_to_configuration(positions)
                moveit2.wait_until_executed()
                time.sleep(0.3)
                query_fk()

            else:
                try:
                    positions = [float(x.strip()) for x in choice.split(',')]
                    if len(positions) != 6:
                        print("❌ 需要6个角度")
                        continue
                    node.get_logger().info(f"移动到: {[f'{p:.2f}' for p in positions]}")
                    moveit2.move_to_configuration(positions)
                    moveit2.wait_until_executed()
                    time.sleep(0.3)
                    query_fk()
                except ValueError:
                    print("❌ 格式错误")

    except KeyboardInterrupt:
        print("\n用户中断")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
