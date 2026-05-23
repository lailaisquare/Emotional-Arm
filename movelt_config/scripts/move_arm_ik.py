#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
逆运动学 (IK)：给定末端位置，自动求解关节角度并执行
"""

import time
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State


def main():
    rclpy.init()
    node = Node('arm_ik_controller')

    joint_names = ['l1', 'l2', 'l3', 'l4', 'l5', 'l6']

    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name='base_link',
        end_effector_name='lamp_link_1',
        group_name='arm_group',
    )
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5
    time.sleep(1.0)

    node.get_logger().info("✅ 逆运动学求解器已就绪")

    print("\n" + "=" * 50)
    print("  🧮 逆运动学 (IK)")
    print("    给定末端位置 → 自动求解关节角度 → 执行")
    print("=" * 50)
    print("  输入末端目标位置 (x y z)")
    print("  输入 q 退出")
    print("=" * 50 + "\n")

    try:
        while rclpy.ok():
            print("\n输入目标位置 (单位: 米):")
            try:
                x_str = input("  X (前后): ").strip()
                if x_str.lower() == 'q':
                    break
                y_str = input("  Y (左右): ").strip()
                if y_str.lower() == 'q':
                    break
                z_str = input("  Z (上下): ").strip()
                if z_str.lower() == 'q':
                    break

                x = float(x_str)
                y = float(y_str)
                z = float(z_str)

                node.get_logger().info(f"🔍 求解 IK: x={x:.2f}, y={y:.2f}, z={z:.2f}")

                moveit2.move_to_pose(
                    position=[x, y, z],
                    quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                    cartesian=False
                )

                time.sleep(0.5)
                state = moveit2.query_state()

                if state == MoveIt2State.IDLE:
                    time.sleep(0.3)
                    state = moveit2.query_state()

                if state == MoveIt2State.IDLE:
                    node.get_logger().error("❌ IK 求解失败！目标不可达")
                    continue

                node.get_logger().info("⏳ IK 求解成功，正在执行...")
                moveit2.wait_until_executed()
                node.get_logger().info("🎉 IK 执行成功!")

            except ValueError:
                print("❌ 请输入有效数字")

    except KeyboardInterrupt:
        print("\n用户中断")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
