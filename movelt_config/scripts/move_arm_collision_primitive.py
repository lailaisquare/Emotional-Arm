#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
添加/清除基本几何体碰撞物体
支持：方块、球体、圆柱体，以及即时清除
"""

import time
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningScene


def main():
    rclpy.init()
    node = Node('arm_collision_controller')

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

    # 碰撞物体发布器
    collision_pub = node.create_publisher(CollisionObject, '/collision_object', 10)

    # 规划场景差分公司
    planning_scene_diff_pub = node.create_publisher(PlanningScene, '/planning_scene', 10)

    # 规划场景服务客户端（用于即时清除）
    apply_planning_scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')

    time.sleep(0.5)

    node.get_logger().info("✅ 碰撞物体控制器已就绪")

    print("\n" + "=" * 50)
    print("  🧱 碰撞物体管理")
    print("=" * 50)
    print("  1 - 添加方块 (Box)")
    print("  2 - 添加球体 (Sphere)")
    print("  3 - 添加圆柱体 (Cylinder)")
    print("  4 - 清除所有碰撞物体")
    print("  q - 退出")
    print("=" * 50 + "\n")

    object_ids = []  # 记录已添加的物体 ID

    try:
        while rclpy.ok():
            choice = input("请输入选项: ").strip().lower()

            if choice == 'q':
                break

            elif choice in ['1', '2', '3']:
                try:
                    x = float(input("  X 位置 (米): "))
                    y = float(input("  Y 位置 (米): "))
                    z = float(input("  Z 位置 (米): "))
                except ValueError:
                    print("❌ 输入格式错误")
                    continue

                obj_id = f"obj_{len(object_ids)}"
                object_ids.append(obj_id)

                collision_obj = CollisionObject()
                collision_obj.header.frame_id = 'base_link'
                collision_obj.id = obj_id

                obj_pose = Pose()
                obj_pose.position.x = x
                obj_pose.position.y = y
                obj_pose.position.z = z
                obj_pose.orientation.w = 1.0

                primitive = SolidPrimitive()

                if choice == '1':
                    primitive.type = SolidPrimitive.BOX
                    size = float(input("  方块边长 (米, 默认0.06): ") or 0.06)
                    primitive.dimensions = [size, size, size]
                    print(f"  🧊 添加方块: {obj_id}")

                elif choice == '2':
                    primitive.type = SolidPrimitive.SPHERE
                    radius = float(input("  球体半径 (米, 默认0.05): ") or 0.05)
                    primitive.dimensions = [radius]
                    print(f"  ⚽ 添加球体: {obj_id}")

                elif choice == '3':
                    primitive.type = SolidPrimitive.CYLINDER
                    height = float(input("  圆柱高度 (米, 默认0.10): ") or 0.10)
                    radius = float(input("  圆柱半径 (米, 默认0.03): ") or 0.03)
                    primitive.dimensions = [height, radius]
                    print(f"  🥫 添加圆柱体: {obj_id}")

                collision_obj.primitives.append(primitive)
                collision_obj.primitive_poses.append(obj_pose)
                collision_obj.operation = CollisionObject.ADD

                # 发布多次确保接收
                for _ in range(5):
                    collision_pub.publish(collision_obj)
                    time.sleep(0.05)

                node.get_logger().info(f"✅ {obj_id} 已添加到规划场景!")

            elif choice == '4':
                # 方法1：逐个移除所有已添加的物体
                for obj_id in object_ids:
                    remove_obj = CollisionObject()
                    remove_obj.header.frame_id = 'base_link'
                    remove_obj.id = obj_id
                    remove_obj.operation = CollisionObject.REMOVE
                    for _ in range(3):
                        collision_pub.publish(remove_obj)
                        time.sleep(0.05)

                # 方法2：通过 planning_scene 差分公司直接清除整个世界
                # 创建一个空的规划场景，替换所有碰撞物体
                clear_scene = PlanningScene()
                clear_scene.is_diff = True
                clear_scene.world.collision_objects = []  # 空列表，表示清除所有

                # 发布多次确保接收
                for _ in range(5):
                    planning_scene_diff_pub.publish(clear_scene)
                    time.sleep(0.05)

                object_ids = []
                node.get_logger().info("🧹 所有碰撞物体已清除!")
                print("   (如果 RViz 中还没消失，请稍等片刻或拖拽一下机械臂刷新)")

            else:
                print("❌ 无效选项")

    except KeyboardInterrupt:
        print("\n用户中断")

    # 退出时清理
    for obj_id in object_ids:
        remove_obj = CollisionObject()
        remove_obj.header.frame_id = 'base_link'
        remove_obj.id = obj_id
        remove_obj.operation = CollisionObject.REMOVE
        collision_pub.publish(remove_obj)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
