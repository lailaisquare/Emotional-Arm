#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线轨迹绘图器（单图版） —— 所有关节角度绘制在同一张图上
"""

import json
import os
import matplotlib.pyplot as plt
import numpy as np


# ======================== 手动设置存放路径 ========================
RECORD_DIR = "/home/lai/Desktop/ros2_ws/src/movelt_config/scripts/records"
# ==================================================================


def load_trajectory(filepath):
    """读取轨迹文件，返回点列表、关节名列表、总时长"""
    with open(filepath) as f:
        data = json.load(f)
    points = data if isinstance(data, list) else data.get('points', [])
    if not points:
        return None, None, 0
    joint_names = points[0].get('joint_names', [])
    # 计算总时长
    if 'time_from_start' in points[-1]:
        dur = points[-1]['time_from_start'] - points[0].get('time_from_start', 0.0)
    elif 'timestamp' in points[-1]:
        dur = points[-1]['timestamp'] - points[0].get('timestamp', 0.0)
    else:
        dur = 0.0
    return points, joint_names, dur


def list_trajectories(record_dir):
    """列出所有 JSON 轨迹文件"""
    try:
        files = sorted([f for f in os.listdir(record_dir) if f.endswith('.json')])
    except FileNotFoundError:
        print(f"❌ 目录不存在: {record_dir}")
        return []
    if not files:
        print("📭 无轨迹文件")
        return []

    print(f"\n📂 {len(files)} 个轨迹:")
    print("-" * 55)
    for i, f in enumerate(files):
        path = os.path.join(record_dir, f)
        points, _, dur = load_trajectory(path)
        pts = len(points) if points else 0
        with open(path) as fp:
            data = json.load(fp)
        segs = data.get('segments', [])
        gaps = data.get('gaps', [])
        gap_info = ""
        if any(g > 0 for g in gaps):
            gap_info = f"  停顿: {', '.join([f'{g}s' for g in gaps if g > 0])}"
        print(f"  [{i}] {f}")
        print(f"      {pts} 点  {len(segs)} 段  {dur:.2f}s{gap_info}")
    print("-" * 55)
    return files


def plot_trajectory(points, joint_names, title="Joint Trajectory"):
    """将所有关节曲线绘制在同一张图上"""
    if not points or not joint_names:
        print("❌ 无数据可绘制")
        return

    # 提取时间和位置
    times = []
    positions = {name: [] for name in joint_names}

    for pt in points:
        t = pt.get('time_from_start', pt.get('timestamp', 0.0))
        times.append(t)
        for i, name in enumerate(joint_names):
            if i < len(pt['positions']):
                positions[name].append(pt['positions'][i])

    times = np.array(times)

    # 创建单个图形
    plt.figure(figsize=(14, 6))
    colors = plt.cm.tab10(np.linspace(0, 1, len(joint_names)))

    for idx, name in enumerate(joint_names):
        plt.plot(times, positions[name], color=colors[idx], linewidth=1.5, label=name)

    plt.xlabel('Time (s)')
    plt.ylabel('Joint angle (rad)')
    plt.title(title)
    plt.legend(loc='best', ncol=2)
    plt.grid(True, alpha=0.3)
    plt.xlim(times[0], times[-1])
    plt.tight_layout()
    plt.show()


def main():
    record_dir = RECORD_DIR
    os.makedirs(record_dir, exist_ok=True)

    print("\n" + "=" * 50)
    print("  📈 离线轨迹绘图器（单图版）")
    print(f"  📁 路径: {record_dir}")
    print("=" * 50)

    files = list_trajectories(record_dir)
    if not files:
        return

    while True:
        try:
            choice = input("\n请输入轨迹编号 (或 q 退出): ").strip()
            if choice.lower() == 'q':
                break
            idx = int(choice)
            if 0 <= idx < len(files):
                filepath = os.path.join(record_dir, files[idx])
                points, joint_names, dur = load_trajectory(filepath)
                if points:
                    print(f"▶️ 绘制: {files[idx]} ({len(points)} 点, {dur:.2f}s)")
                    plot_trajectory(points, joint_names,
                                    title=f"Trajectory: {files[idx]}")
            else:
                print(f"❌ 请输入 0 到 {len(files)-1}")
        except ValueError:
            print("❌ 输入无效")
        except (EOFError, KeyboardInterrupt):
            break


if __name__ == "__main__":
    main()
