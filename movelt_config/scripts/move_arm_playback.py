#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂轨迹回放器（修复版）
- 自动确保时间戳严格递增，避免控制器拒绝
- 支持多倍速，兼容所有录制版本
- 路径在顶部手动设置
"""

import time
import json
import os
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ======================== 手动设置存放路径 ========================
RECORD_DIR = "/home/lai/Desktop/ros2_ws/src/movelt_config/scripts/records"
# ==================================================================

SPEEDS = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0]


class TrajectoryPlayer(Node):
    def __init__(self, record_dir, controller_name='arm_group_controller'):
        super().__init__('trajectory_player')
        self.publisher = self.create_publisher(
            JointTrajectory,
            f'/{controller_name}/joint_trajectory',
            10
        )
        self.record_dir = record_dir
        os.makedirs(self.record_dir, exist_ok=True)

    def _extract_points(self, data):
        """兼容各种格式，返回纯点列表"""
        if isinstance(data, list):
            return data
        return data.get('points', [])

    def _get_duration(self, points):
        """从点列表计算总时长"""
        if len(points) < 2:
            return 0.0
        if 'timestamp' in points[-1]:
            return points[-1]['timestamp'] - points[0]['timestamp']
        if 'time_from_start' in points[-1]:
            return points[-1]['time_from_start'] - points[0].get('time_from_start', 0.0)
        return 0.0

    def list_trajectories(self):
        files = sorted([f for f in os.listdir(self.record_dir) if f.endswith('.json')])
        if not files:
            print("📭 无轨迹文件")
            return []

        print(f"\n📂 {len(files)} 个轨迹:")
        print("-" * 55)
        for i, f in enumerate(files):
            path = os.path.join(self.record_dir, f)
            with open(path) as fp:
                data = json.load(fp)

            points = self._extract_points(data)
            segs = data.get('segments', [len(points)]) if isinstance(data, dict) else [len(points)]
            gaps = data.get('gaps', []) if isinstance(data, dict) else []
            dur = self._get_duration(points)

            gap_info = ""
            if any(g > 0 for g in gaps):
                gap_info = f"  停顿: {', '.join([f'{g}s' for g in gaps if g > 0])}"

            print(f"  [{i}] {f}")
            print(f"      {len(points)} 点  {len(segs)} 段  {dur:.1f}s{gap_info}")
        print("-" * 55)
        return files

    def load_trajectory(self, filename):
        filepath = os.path.join(self.record_dir, filename)
        if not os.path.exists(filepath):
            self.get_logger().error(f"文件不存在: {filepath}")
            return None
        with open(filepath, 'r') as f:
            data = json.load(f)
        return self._extract_points(data)

    def play_trajectory(self, points, speed=1.0):
        if not points or len(points) < 2:
            self.get_logger().error("轨迹点太少")
            return False

        joint_names = points[0]['joint_names']
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        # 获取时间戳函数（兼容不同格式）
        if 'time_from_start' in points[0]:
            t0 = points[0]['time_from_start']
            get_time = lambda p: p['time_from_start']
        elif 'timestamp' in points[0]:
            t0 = points[0]['timestamp']
            get_time = lambda p: p['timestamp']
        else:
            t0 = 0.0
            get_time = lambda p: p.get('timestamp', p.get('time_from_start', 0.0))

        last_t = -1.0  # 记录上一个点的时间（秒），确保严格递增
        for p in points:
            raw_t = get_time(p) - t0
            dt = raw_t / speed

            # 如果当前时间不严格大于上一个，则强制增加一个极小值
            if dt <= last_t:
                dt = last_t + 0.001  # 增加 1ms
                self.get_logger().warn(
                    f"⚠️ 修正时间戳：原时间 {raw_t/speed:.4f}s → 调整为 {dt:.4f}s"
                )

            pt = JointTrajectoryPoint()
            pt.positions = p['positions']
            pt.time_from_start = Duration(
                sec=int(dt),
                nanosec=int((dt - int(dt)) * 1e9)
            )
            traj_msg.points.append(pt)
            last_t = dt

        real_dur = get_time(points[-1]) - t0
        play_dur = real_dur / speed

        self.get_logger().info(f"▶️  {len(points)} 点  {real_dur:.1f}s → {speed}x → {play_dur:.1f}s")
        self.publisher.publish(traj_msg)
        self.get_logger().info("✅ 已发布")
        return True


def main():
    rclpy.init()

    record_dir = RECORD_DIR
    os.makedirs(record_dir, exist_ok=True)

    player = TrajectoryPlayer(record_dir)

    print("\n" + "=" * 50)
    print("  ▶️ 轨迹回放器（修复版）")
    print(f"  📁 存放路径: {record_dir}")
    print("  输入: 编号 [倍速]")
    print(f"  倍速: {', '.join([f'{s}x' for s in SPEEDS])}")
    print("  l=列表  q=退出")
    print("=" * 50 + "\n")

    try:
        while rclpy.ok():
            files = player.list_trajectories()
            if not files:
                print("请先用 move_arm_record.py 录制轨迹")
                break

            print("\n输入 > ", end='', flush=True)
            choice = input().strip().lower()

            if choice == 'q':
                break
            elif choice == 'l':
                continue
            else:
                parts = choice.split()
                try:
                    idx = int(parts[0])
                    speed = float(parts[1]) if len(parts) > 1 else 1.0

                    if speed not in SPEEDS:
                        closest = min(SPEEDS, key=lambda s: abs(s - speed))
                        print(f"  ⚠️  使用 {closest}x")
                        speed = closest

                    if 0 <= idx < len(files):
                        points = player.load_trajectory(files[idx])
                        if points:
                            player.play_trajectory(points, speed)
                            dur = player._get_duration(points)
                            time.sleep(dur / speed + 1.0)
                    else:
                        print(f"❌ 0-{len(files)-1}")
                except ValueError:
                    print("❌ 格式: 编号 [倍速]")

    except KeyboardInterrupt:
        print("\n中断")
    finally:
        player.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
