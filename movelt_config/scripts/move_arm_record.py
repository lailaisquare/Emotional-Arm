#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂轨迹录制器 v12.1 - 修复旧文件兼容问题
订阅 /display_planned_path + 内置25Hz插值
支持新旧轨迹文件格式，list命令不再崩溃
"""

import time
import json
import os
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass
from typing import List


# ======================== 手动设置 ========================
RECORD_DIR = "/home/lai/Desktop/ros2_ws/src/movelt_config/scripts/records"
TARGET_FREQ = 25.0  # 目标采样频率（25Hz = 60秒1500点）
# ========================================================


@dataclass
class TrajPoint:
    time_from_start: float
    positions: List[float]


def resample_points(points: List[TrajPoint], rate_hz: float) -> List[TrajPoint]:
    """和你播放脚本完全一样的插值函数"""
    if rate_hz <= 0 or len(points) < 2:
        return points

    dt = 1.0 / rate_hz
    start = points[0].time_from_start
    end = points[-1].time_from_start

    frames: List[TrajPoint] = []
    t = start
    i = 0
    while t < end:
        while i < len(points) - 2 and points[i + 1].time_from_start < t:
            i += 1
        p0 = points[i]
        p1 = points[i + 1]
        if p1.time_from_start == p0.time_from_start:
            alpha = 0.0
        else:
            alpha = (t - p0.time_from_start) / (p1.time_from_start - p0.time_from_start)
        positions = [
            p0.positions[j] + alpha * (p1.positions[j] - p0.positions[j])
            for j in range(len(p0.positions))
        ]
        frames.append(TrajPoint(time_from_start=t, positions=positions))
        t += dt

    frames.append(points[-1])
    return frames


class TrajectoryRecorder(Node):
    def __init__(self, record_dir):
        super().__init__('trajectory_recorder')

        self.display_sub = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.display_traj_callback,
            10
        )

        self.is_recording = False
        self.last_planned_trajectory = None
        self.recorded_segments = []
        self.segment_gaps = []

        self.record_dir = record_dir
        os.makedirs(self.record_dir, exist_ok=True)

    def display_traj_callback(self, msg):
        if not self.is_recording or len(msg.trajectory) == 0:
            return
        traj = msg.trajectory[0]
        jt = traj.joint_trajectory
        if len(jt.points) == 0:
            return

        # 先转成TrajPoint格式
        raw_points = []
        joint_names = list(jt.joint_names)
        for pt in jt.points:
            t_offset = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            raw_points.append(TrajPoint(
                time_from_start=t_offset,
                positions=list(pt.positions)
            ))

        # ✅ 关键：插值到目标频率
        dense_points = resample_points(raw_points, TARGET_FREQ)

        # 转成保存格式
        segment = []
        for pt in dense_points:
            segment.append({
                'positions': pt.positions,
                'time_offset': pt.time_from_start,
                'joint_names': joint_names
            })

        self.recorded_segments.append(segment)
        self.segment_gaps.append(0.0)
        n = len(self.recorded_segments)
        pts = len(segment)
        self.get_logger().info(f"✅ 第 {n} 段  原始{len(raw_points)}点 → 插值后{pts}点  频率:{TARGET_FREQ}Hz")

    def set_gap(self, seconds):
        if len(self.segment_gaps) > 0:
            self.segment_gaps[-1] = seconds
            self.get_logger().info(f"⏸️  第 {len(self.segment_gaps)} 段后停顿 {seconds}s")

    def start_recording(self):
        self.is_recording = True
        self.recorded_segments = []
        self.segment_gaps = []
        self.get_logger().info(f"🔴 开始录制（自动插值到 {TARGET_FREQ}Hz）")

    def stop_recording(self):
        self.is_recording = False

        if len(self.recorded_segments) == 0:
            self.get_logger().warn("⚠️  无轨迹，请在RViz中点击Plan&Execute")
            return

        all_points = []
        time_cursor = 0.0

        for i, seg in enumerate(self.recorded_segments):
            if len(seg) == 0:
                continue
            for pt in seg:
                all_points.append({
                    'time_from_start': time_cursor + pt['time_offset'],
                    'positions': pt['positions'],
                    'joint_names': pt['joint_names']
                })
            time_cursor += seg[-1]['time_offset']

            gap = self.segment_gaps[i] if i < len(self.segment_gaps) else 0.0
            if gap > 0 and i < len(self.recorded_segments) - 1:
                last_pt = seg[-1]
                t = time_cursor + 1.0/TARGET_FREQ
                while t <= time_cursor + gap:
                    all_points.append({
                        'time_from_start': t,
                        'positions': last_pt['positions'],
                        'joint_names': last_pt['joint_names']
                    })
                    t += 1.0/TARGET_FREQ
                time_cursor += gap + 1.0/TARGET_FREQ

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{timestamp}.json"
        filepath = os.path.join(self.record_dir, filename)

        save_data = {
            'segments': [len(s) for s in self.recorded_segments],
            'gaps': self.segment_gaps,
            'total_points': len(all_points),
            'points': all_points
        }

        with open(filepath, 'w') as f:
            json.dump(save_data, f, indent=2)

        dur = all_points[-1]['time_from_start'] - all_points[0]['time_from_start']
        self.get_logger().info(f"💾 {len(self.recorded_segments)} 段  总{len(all_points)}点  总时长{dur:.1f}s")
        self.get_logger().info(f"📁 {filepath}")

    def list_trajectories(self):
        os.makedirs(self.record_dir, exist_ok=True)
        files = sorted([f for f in os.listdir(self.record_dir) if f.endswith('.json')])
        if not files:
            print("📭 无轨迹文件")
            return files
        print(f"\n📂 {len(files)} 个轨迹:")
        for i, f in enumerate(files):
            path = os.path.join(self.record_dir, f)
            try:
                with open(path) as fp:
                    data = json.load(fp)
                pts = data.get('total_points', 0)
                segs = data.get('segments', [])
                gaps = data.get('gaps', [])
                dur = 0
                points = data.get('points', [])
                if len(points) > 1:
                    # ✅ 修复：同时支持新旧两种时间字段
                    t0 = points[0].get('time_from_start', points[0].get('timestamp', 0))
                    t1 = points[-1].get('time_from_start', points[-1].get('timestamp', 0))
                    dur = t1 - t0
                gap_info = ", ".join([f"{g}s" for g in gaps if g > 0]) if any(g > 0 for g in gaps) else "无停顿"
                print(f"  [{i}] {f}")
                print(f"      {pts} 点  {len(segs)} 段  {dur:.1f}s  停顿: {gap_info}")
            except Exception as e:
                print(f"  [{i}] {f}")
                print(f"      ⚠️  文件损坏或格式错误: {str(e)}")
        return files


def main():
    rclpy.init()

    record_dir = RECORD_DIR
    os.makedirs(record_dir, exist_ok=True)

    recorder = TrajectoryRecorder(record_dir)

    print("\n" + "=" * 50)
    print("  🎬 轨迹录制器 v12.1 - 修复旧文件兼容")
    print(f"  📁 存放路径: {record_dir}")
    print(f"  🎯 目标频率: {TARGET_FREQ}Hz (60秒={int(60*TARGET_FREQ)}点)")
    print("  r=录制  l=列表  q=退出")
    print("  数字=设置段间停顿秒数 (如 2 = 停2秒)")
    print("=" * 50 + "\n")

    recording = False
    try:
        while rclpy.ok():
            rclpy.spin_once(recorder, timeout_sec=0.05)

            if recording:
                segs = len(recorder.recorded_segments)
                total = sum(len(s) for s in recorder.recorded_segments)
                gaps = recorder.segment_gaps
                gap_str = ""
                if gaps and gaps[-1] > 0:
                    gap_str = f"  段间停顿: {gaps[-1]}s"
                print(f"\r🔴 {total} 点  {segs} 段{gap_str}  |  r=停止  数字=停顿秒数", end='', flush=True)
            else:
                print(f"\r⏸️  待机  |  r=录制  l=列表  q=退出", end='', flush=True)

            import sys, select
            if select.select([sys.stdin], [], [], 0.05)[0]:
                choice = sys.stdin.readline().strip().lower()

                if recording:
                    if choice == 'r':
                        recorder.stop_recording()
                        recording = False
                    elif choice == 'q':
                        recorder.stop_recording()
                        break
                    else:
                        try:
                            gap = float(choice)
                            if gap >= 0:
                                recorder.set_gap(gap)
                        except ValueError:
                            pass
                else:
                    if choice == 'r':
                        recorder.start_recording()
                        recording = True
                    elif choice == 'l':
                        recorder.list_trajectories()
                    elif choice == 'q':
                        break

    except KeyboardInterrupt:
        pass
    finally:
        if recording:
            recorder.stop_recording()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
