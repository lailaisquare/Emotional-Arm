#!/usr/bin/env python3
"""
真实机械臂轨迹回放器 —— 高可靠性版
- 自动复位串口信号，连续运行不会失败
- 支持多次退避重试，适应 USB 芯片释放延迟
"""

import time, json, os, argparse, sys
from pathlib import Path
from typing import List, Tuple
import serial

HEADER = b"\xFF\xFF"
INST_WRITE = 0x03

def checksum(data: bytes) -> int:
    return (~sum(data) & 0xFF)

class FeetechBus:
    def __init__(self, port: str, baudrate: int, timeout: float = 0.05):
        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            timeout=timeout, write_timeout=timeout
        )

    def close(self):
        if self.ser.is_open:
            self.ser.flush()
            try:
                self.ser.setDTR(False)
                self.ser.setRTS(False)
            except:
                pass
            self.ser.close()
            time.sleep(0.5)          # 关键：确保内核完全释放端口

    def _build_packet(self, servo_id, instruction, params):
        length = len(params) + 2
        body = bytes([servo_id, length, instruction]) + params
        return HEADER + body + bytes([checksum(body)])

    def write_register(self, servo_id, addr, data, wait_status=False):
        packet = self._build_packet(servo_id, INST_WRITE, bytes([addr]) + data)
        self.ser.reset_input_buffer()
        self.ser.write(packet)
        self.ser.flush()
        return True, "ok"

def degree_to_raw(degree: float, raw_max: int = 4095) -> int:
    degree = degree % 360.0
    raw = int(round((degree / 360.0) * raw_max))
    return max(0, min(raw_max, raw))

def joint_angle_to_servo_angle(joint_deg: float) -> float:
    servo_angle = 180.0 + joint_deg
    return max(0.0, min(360.0, servo_angle))

def load_trajectory(filepath: str) -> List[dict]:
    with open(filepath) as f:
        data = json.load(f)
    if isinstance(data, list):
        return data
    return data.get('points', [])

def list_trajectories(record_dir: str):
    files = sorted([f for f in os.listdir(record_dir) if f.endswith('.json')])
    if not files:
        print("📭 无轨迹文件")
        return []
    print(f"\n📂 {len(files)} 个轨迹:")
    print("-" * 55)
    for i, f in enumerate(files):
        path = os.path.join(record_dir, f)
        with open(path) as fp:
            data = json.load(fp)
        points = data if isinstance(data, list) else data.get('points', [])
        dur = 0.0
        if len(points) >= 2:
            if 'time_from_start' in points[-1]:
                dur = points[-1]['time_from_start'] - points[0].get('time_from_start', 0.0)
            elif 'timestamp' in points[-1]:
                dur = points[-1]['timestamp'] - points[0].get('timestamp', 0.0)
        print(f"  [{i}] {f}  ({len(points)} 点, {dur:.2f}s)")
    print("-" * 55)
    return files

def find_records_dir():
    script_dir = Path(__file__).resolve().parent
    alt = script_dir.parents[1] / 'movelt_config' / 'scripts' / 'records'
    if alt.is_dir():
        return str(alt)
    return str(Path.cwd() / 'records')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1000000)
    parser.add_argument("--ids", type=int, nargs=6, default=[1,2,3,4,5,6])
    parser.add_argument("--record-dir", default=None)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--file", default=None)
    parser.add_argument("--index", type=int, default=None)
    parser.add_argument("--move-time-min", type=int, default=100)
    args = parser.parse_args()

    record_dir = args.record_dir or find_records_dir()
    print(f"📁 记录目录: {record_dir}")

    if args.file:
        filepath = os.path.join(record_dir, args.file)
        if not os.path.exists(filepath):
            print("❌ 文件不存在")
            return
    elif args.index is not None:
        files = list_trajectories(record_dir)
        if not files: return
        if 0 <= args.index < len(files):
            filepath = os.path.join(record_dir, files[args.index])
        else:
            print(f"❌ 索引 0-{len(files)-1}")
            return
    else:
        files = list_trajectories(record_dir)
        if not files: return
        while True:
            try:
                choice = input("请输入轨迹编号 (或 q 退出): ").strip()
                if choice.lower() == 'q': return
                idx = int(choice)
                if 0 <= idx < len(files):
                    filepath = os.path.join(record_dir, files[idx])
                    break
                else:
                    print(f"0-{len(files)-1}")
            except ValueError:
                print("输入无效")
            except (EOFError, KeyboardInterrupt):
                return

    points = load_trajectory(filepath)
    if len(points) < 2:
        print("❌ 轨迹点太少")
        return

    joint_names = points[0].get('joint_names', ['l1','l2','l3','l4','l5','l6'])
    id_map = {joint_names[i]: args.ids[i] for i in range(min(len(joint_names),6))}
    print(f"🔧 映射: {id_map}  速度: {args.speed}x  点数: {len(points)}")

    # ---------- 高可靠串口连接 ----------
    bus = None
    for attempt in range(3):
        try:
            bus = FeetechBus(args.port, args.baudrate)
            print("✅ 串口已连接")
            break
        except Exception as e:
            wait = 1 * (attempt + 1)
            if attempt < 2:
                print(f"⚠️ 打开失败 ({e})，{wait}秒后重试...")
                try:  # 尝试关闭可能残留的句柄
                    s = serial.Serial(args.port)
                    s.close()
                except: pass
                time.sleep(wait)
            else:
                print(f"❌ 无法打开串口: {e}")
                print("   请执行: sudo lsof /dev/ttyUSB0 查看占用")
                return

    try:
        if 'time_from_start' in points[0]:
            t0 = points[0]['time_from_start']
            get_time = lambda p: p['time_from_start']
        elif 'timestamp' in points[0]:
            t0 = points[0]['timestamp']
            get_time = lambda p: p['timestamp']
        else:
            t0 = 0.0
            get_time = lambda p: p.get('timestamp', p.get('time_from_start', 0.0))

        prev_time = get_time(points[0]) / args.speed
        prev_positions = points[0]['positions']

        print("📍 移动至起始点...")
        for i, joint in enumerate(joint_names):
            if i >= len(prev_positions): break
            jd = prev_positions[i] * 180.0 / 3.14159265
            sa = joint_angle_to_servo_angle(jd)
            raw = degree_to_raw(sa)
            sid = id_map[joint]
            data = raw.to_bytes(2,'little') + (1000).to_bytes(2,'little') + (0).to_bytes(2,'little')
            bus.write_register(sid, 42, data)
            time.sleep(0.005)
        time.sleep(1.2)

        print("▶️ 轨迹回放中...")
        for idx in range(1, len(points)):
            point = points[idx]
            cur_time = get_time(point) / args.speed
            dt = cur_time - prev_time
            if dt <= 0: continue
            move_ms = max(int(dt * 1000), args.move_time_min)
            positions = point['positions']

            for i, joint in enumerate(joint_names):
                if i >= len(positions): break
                jd = positions[i] * 180.0 / 3.14159265
                sa = joint_angle_to_servo_angle(jd)
                raw = degree_to_raw(sa)
                sid = id_map[joint]
                data = raw.to_bytes(2,'little') + move_ms.to_bytes(2,'little') + (0).to_bytes(2,'little')
                bus.write_register(sid, 42, data)
                time.sleep(0.005)
            time.sleep(dt)
            prev_time = cur_time

        print("✅ 回放完成")
    except KeyboardInterrupt:
        print("\n⏸️ 中断")
    finally:
        if bus:
            bus.close()
            print("🔌 串口已关闭")

if __name__ == "__main__":
    main()
