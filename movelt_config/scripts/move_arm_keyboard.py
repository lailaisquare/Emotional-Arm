#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘实时控制机械臂各关节
支持连续按键，自动取消旧轨迹

按键布局：
  l1:  Q (+)  /  A (-)
  l2:  W (+)  /  S (-)
  l3:  E (+)  /  D (-)
  l4:  R (+)  /  F (-)
  l5:  T (+)  /  G (-)
  l6:  Y (+)  /  H (-)

  归零: 0    显示角度: P    退出: ESC
"""

import time
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import sys
import tty
import termios
import select
import math


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], 0.03)
        if ready:
            return sys.stdin.read(1)
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = Node('arm_keyboard_controller')

    joint_names = ['l1', 'l2', 'l3', 'l4', 'l5', 'l6']

    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name='base_link',
        end_effector_name='lamp_link_1',
        group_name='arm_group',
    )
    moveit2.max_velocity = 0.8
    moveit2.max_acceleration = 0.8

    # 初始化归零
    node.get_logger().info("🔄 正在初始化 (归零)...")
    current_positions = [0.0] * 6
    time.sleep(1.5)
    moveit2.move_to_configuration(current_positions)
    moveit2.wait_until_executed()
    time.sleep(0.5)
    node.get_logger().info("✅ 准备就绪")

    DEG_TO_RAD = math.pi / 180.0
    step = 1.0 * DEG_TO_RAD

    key_joint_map = {
        'q': 0, 'a': 0,
        'w': 1, 's': 1,
        'e': 2, 'd': 2,
        'r': 3, 'f': 3,
        't': 4, 'g': 4,
        'y': 5, 'h': 5,
    }

    positive_keys = set('qwerty')

    print("\n" + "=" * 50)
    print("  🎮 键盘控制 (±1°)")
    print("=" * 50)
    print("  l1: Q(+)│A(-)   l2: W(+)│S(-)")
    print("  l3: E(+)│D(-)   l4: R(+)│F(-)")
    print("  l5: T(+)│G(-)   l6: Y(+)│H(-)")
    print("  归零: 0 | 显示: P | 退出: ESC")
    print("=" * 50)

    def print_status():
        print(f"\n{'─'*30}")
        for name, val in zip(joint_names, current_positions):
            print(f"  {name}: {val*180.0/math.pi:+.1f}°")
        print(f"{'─'*30}")

    print_status()

    last_move_time = 0
    move_interval = 0.08  # 发指令的最小间隔

    try:
        while rclpy.ok():
            key = get_key()
            if key is None:
                continue

            if ord(key) == 27:
                break

            if key == '0':
                current_positions = [0.0] * 6
                moveit2.cancel_execution()  # 取消当前运动
                time.sleep(0.05)
                moveit2.move_to_configuration(current_positions)
                moveit2.wait_until_executed()
                print("✅ 已归零")
                print_status()
                continue

            if key.lower() == 'p':
                print_status()
                continue

            key_lower = key.lower()
            if key_lower in key_joint_map:
                now = time.time()
                if now - last_move_time < move_interval:
                    continue
                last_move_time = now

                joint_idx = key_joint_map[key_lower]
                delta = +step if key_lower in positive_keys else -step
                current_positions[joint_idx] += delta

                # 取消当前正在执行的轨迹，再发新的
                moveit2.cancel_execution()
                time.sleep(0.03)
                moveit2.move_to_configuration(current_positions)

    except KeyboardInterrupt:
        pass

    finally:
        moveit2.cancel_execution()
        time.sleep(0.1)
        rclpy.shutdown()
        print("\n已退出")


if __name__ == '__main__':
    main()
