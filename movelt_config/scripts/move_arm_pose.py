#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
笛卡尔空间运动：通过 MoveIt2 的 move_to_pose 规划，然后检测执行结果
"""

import time
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2, MoveIt2State


def main():
    rclpy.init()
    node = Node('arm_pose_controller')

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

    node.get_logger().info("✅ 笛卡尔空间控制器已就绪")

    print("\n" + "=" * 50)
    print("  📍 末端位姿控制")
    print("=" * 50)
    print("  0 - 预设位置 1 (前方 0.30, 0.00, 0.30)")
    print("  1 - 预设位置 2 (上方 0.15, 0.00, 0.50)")
    print("  2 - 预设位置 3 (右侧 0.3, 0.20, 0.30)")
    print("  3 - 自定义位置 (输入 x y z)")
    print("  q - 退出")
    print("=" * 50 + "\n")

    def try_move_to_pose(x, y, z):
        node.get_logger().info(f"🎯 目标位置: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # 发送笛卡尔位姿规划
        moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            cartesian=False
        )

        # 等待一小段时间，让规划器响应
        time.sleep(0.5)

        # 查询状态
        state = moveit2.query_state()

        # 如果是 IDLE 且没在动，说明规划直接被拒了
        if state == MoveIt2State.IDLE:
            # 再等一下确认
            time.sleep(0.3)
            state = moveit2.query_state()

        if state == MoveIt2State.IDLE:
            node.get_logger().error("❌ 规划失败！目标位置不可达")
            print("   可能原因:")
            print("     • 目标位置超出工作空间范围")
            print("     • IK 求解器 (KDL) 无法找到解")
            print("   建议: 试试选项 0 或更近的位置")
            return False

        # 正在执行
        node.get_logger().info("⏳ 规划成功，正在执行轨迹...")
        moveit2.wait_until_executed()
        node.get_logger().info("🎉 运动执行完成!")
        return True

    try:
        while rclpy.ok():
            choice = input("\n请输入选项: ").strip().lower()

            if choice == 'q':
                break

            x, y, z = 0.0, 0.0, 0.0

            if choice == '0':
                x, y, z = 0.30, 0.00, 0.30
            elif choice == '1':
                x, y, z = 0.15, 0.00, 0.50
            elif choice == '2':
                x, y, z = 0.30, 0.20, 0.30
            elif choice == '3':
                try:
                    x = float(input("  X (前后, 米): ").strip())
                    y = float(input("  Y (左右, 米): ").strip())
                    z = float(input("  Z (上下, 米): ").strip())
                except ValueError:
                    print("❌ 输入格式错误")
                    continue
            else:
                print("❌ 无效选项")
                continue

            try_move_to_pose(x, y, z)

    except KeyboardInterrupt:
        print("\n用户中断")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
