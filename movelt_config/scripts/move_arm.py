#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机械臂控制脚本 - 基于 pymoveit2
简化版，去掉多线程 executor
"""

import time
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2


def main():
    rclpy.init()

    node = Node('arm_controller')

    # 你的机械臂参数
    joint_names = ['l1', 'l2', 'l3', 'l4', 'l5', 'l6']
    base_link = 'base_link'
    end_effector = 'lamp_link_1'
    group_name = 'arm_group'

    # 创建 MoveIt2 接口
    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name=base_link,
        end_effector_name=end_effector,
        group_name=group_name,
    )

    # 设置速度和加速度
    moveit2.max_velocity = 0.3
    moveit2.max_acceleration = 0.3

    # 等一小会儿让它初始化
    time.sleep(1.0)

    node.get_logger().info("✅ 机械臂控制器已就绪")

    print("\n" + "=" * 50)
    print("  🤖 你的机械臂 - Python 控制程序")
    print("=" * 50)
    print("  0 - 回到 home 位置 (全零)")
    print("  1 - 预设置姿态 1 (微弯曲)")
    print("  2 - 预设置姿态 2 (向后伸)")
    print("  3 - 预设置姿态 3 (回收)")
    print("  4 - 自定义关节角度")
    print("  q - 退出")
    print("=" * 50 + "\n")

    try:
        while rclpy.ok():
            choice = input("请输入选项: ").strip().lower()

            if choice == '0':
                positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                node.get_logger().info(f"回到 home: {positions}")
                moveit2.move_to_configuration(positions)
                moveit2.wait_until_executed()

            elif choice == '1':
                positions = [0.0, -0.3, 0.5, 0.0, 0.3, 0.0]
                node.get_logger().info(f"姿态1: {positions}")
                moveit2.move_to_configuration(positions)
                moveit2.wait_until_executed()

            elif choice == '2':
                positions = [0.0, -0.8, 1.2, 0.0, 0.6, 0.0]
                node.get_logger().info(f"姿态2: {positions}")
                moveit2.move_to_configuration(positions)
                moveit2.wait_until_executed()

            elif choice == '3':
                positions = [0.0, 0.2, -0.4, 0.0, -0.3, 0.0]
                node.get_logger().info(f"姿态3: {positions}")
                moveit2.move_to_configuration(positions)
                moveit2.wait_until_executed()

            elif choice == '4':
                try:
                    pos_str = input("输入6个关节角度，用逗号分隔 (如: 0.0, -0.5, 0.5, 0.0, 0.5, 0.0): ")
                    positions = [float(x.strip()) for x in pos_str.split(',')]
                    if len(positions) != 6:
                        print("❌ 需要精确6个关节角度！")
                        continue
                    node.get_logger().info(f"自定义: {positions}")
                    moveit2.move_to_configuration(positions)
                    moveit2.wait_until_executed()
                except ValueError:
                    print("❌ 输入格式错误，请输入数字")

            elif choice == 'q':
                break

            else:
                print("❌ 无效选项，请重新输入")

    except KeyboardInterrupt:
        print("\n用户中断")

    node.get_logger().info("关闭...")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
