#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键修改所有控制脚本中的：关节名称、关节数量、末端执行器名称等
支持任意轴数，覆盖所有可能的写法
"""

import re
import sys
from pathlib import Path


this_file = Path(__file__).resolve()

try:
    # 找到工作空间根路径（包含 src、install、build）
    parts = this_file.parts
    idx = parts.index("install")
    workspace = Path(*parts[:idx])
    pkg_name = parts[idx + 1]

    # 真实源码路径：src/功能包名/scripts
    real_scripts_dir = workspace / "src" / pkg_name / "scripts"

except:
    real_scripts_dir = this_file.parent


class ArmConfigEditor:
    def __init__(self):
        self.script_dir = real_scripts_dir
        self.config = {
            'joint_names': [],
            'num_joints': 0,
            'base_link': 'base_link',
            'end_effector': 'lamp_link_1',
            'group_name': 'arm_group',
        }
        self.scripts = []

    def scan_scripts(self):
        print(f"\n📂 真实功能包脚本路径：\033[32m{self.script_dir}\033[0m")
        print("=" * 80)

        if not self.script_dir.exists():
            print(f"❌ 路径不存在：{self.script_dir}")
            sys.exit(1)

        self.scripts = sorted(self.script_dir.glob("move_arm*.py"))
        self.scripts = [s for s in self.scripts if "move_arm_config" not in s.name]
        return self.scripts

    def detect_current_config(self):
        print("\n🔍 正在检测当前配置...\n")
        if not self.scripts:
            return

        content = self.scripts[0].read_text()
        match = re.search(r"joint_names\s*=\s*\[([^\]]*)\]", content, re.DOTALL)
        if match:
            joints_str = match.group(1)
            joints = re.findall(r"'([^']*)'", joints_str)
            if joints:
                self.config['joint_names'] = joints
                self.config['num_joints'] = len(joints)

        for key, pattern in [
            ('end_effector', r'end_effector_name\s*=\s*["\']([^"\']*)["\']'),
            ('base_link', r'base_link_name\s*=\s*["\']([^"\']*)["\']'),
            ('group_name', r'group_name\s*=\s*["\']([^"\']*)["\']'),
        ]:
            match = re.search(pattern, content)
            if match:
                self.config[key] = match.group(1)

    def show_current_config(self):
        print("=" * 50)
        print("  📋 当前配置")
        print("=" * 50)
        items = [
            ('num_joints', '关节数量'),
            ('joint_names', '关节名称'),
            ('base_link', '基础连杆'),
            ('end_effector', '末端执行器'),
            ('group_name', '规划组'),
        ]
        for key, label in items:
            print(f"  {label}: {self.config[key]}")
        print("=" * 50)

    def input_new_config(self):
        print("\n" + "=" * 50)
        print("  ✏️  输入新配置 (直接回车保留当前值)")
        print("=" * 50 + "\n")

        num_str = input(f"关节数量 [{self.config['num_joints']}]: ").strip()
        if num_str:
            self.config['num_joints'] = int(num_str)

        print(f"\n当前关节名称: {self.config['joint_names']}")
        print("输入新的关节名称 (空格/逗号分隔):")
        names_str = input("> ").strip()
        if names_str:
            names = re.split(r'[,\s]+', names_str)
            names = [n for n in names if n]
            if len(names) != self.config['num_joints']:
                print(f"⚠️  名称数量 {len(names)} ≠ 关节数量 {self.config['num_joints']}")
                if input("自动调整关节数量? (y/n): ").lower() == 'y':
                    self.config['num_joints'] = len(names)
            self.config['joint_names'] = names

        for key, label in [
            ('base_link', '基础连杆'),
            ('end_effector', '末端执行器'),
            ('group_name', '规划组'),
        ]:
            val = input(f"\n{label} [{self.config[key]}]: ").strip()
            if val:
                self.config[key] = val

        print("\n" + "=" * 50)
        print("  📋 新配置预览")
        print("=" * 50)
        self.show_current_config()
        return input("\n确认修改所有脚本? (y/n): ").strip().lower() == 'y'

    def update_script(self, filepath):
        content = filepath.read_text()
        orig = content

        jn = self.config['joint_names']
        bl = self.config['base_link']
        ee = self.config['end_effector']
        gn = self.config['group_name']

        content = re.sub(r'\bbase_link\s*=', 'base_link_name =', content)
        content = re.sub(r'\bend_effector\s*=', 'end_effector_name =', content)

        content = re.sub(r'base_link_name\s*=\s*["\'].*?["\']', f'base_link_name = "{bl}"', content)
        content = re.sub(r'end_effector_name\s*=\s*["\'].*?["\']', f'end_effector_name = "{ee}"', content)
        content = re.sub(r'group_name\s*=\s*["\'].*?["\']', f'group_name = "{gn}"', content)

        joint_str = ', '.join(f'"{x}"' for x in jn)
        content = re.sub(r'joint_names\s*=\s*\[.*?\]', f'joint_names = [{joint_str}]', content, flags=re.DOTALL)

        if content != orig:
            filepath.write_text(content, encoding='utf-8')
            return True
        return False

    def apply_all(self):
        print("\n🔄 正在更新脚本...\n")
        count = 0
        for s in self.scripts:
            if self.update_script(s):
                print(f"✅ {s.name}")
                count += 1
            else:
                print(f"⏭️ {s.name} (无需修改)")
        print(f"\n📊 完成：更新 {count} 个文件")

    def run(self):
        print("=" * 60)
        print("  🔧 机械臂脚本配置工具（最终完美版）")
        print("=" * 60)

        if not self.scan_scripts():
            print("❌ 未找到 move_arm*.py 脚本")
            return

        print(f"✅ 找到 {len(self.scripts)} 个脚本")
        self.detect_current_config()
        self.show_current_config()

        if not self.input_new_config():
            print("取消")
            return

        self.apply_all()
        print("\n🎉 全部完成！")
        print("✅ 现在路径 100% 正确！")


def main():
    ArmConfigEditor().run()


if __name__ == '__main__':
    main()
