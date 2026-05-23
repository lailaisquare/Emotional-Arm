from setuptools import setup
import os
from pathlib import Path

package_name = 'my_arm_description'

def find_files(directory):
    """递归返回目录下所有文件的相对路径"""
    return [str(p) for p in Path(directory).rglob('*') if p.is_file()]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # launch 文件（直接放在 launch/ 下）
        (os.path.join('share', package_name, 'launch'),
         [str(p) for p in Path('launch').glob('*.launch.py')]),
        
        # urdf 子目录（保持目录结构）
        (os.path.join('share', package_name, 'urdf', 'core'),
         find_files('urdf/core')),
        (os.path.join('share', package_name, 'urdf', 'gazebo'),
         find_files('urdf/gazebo')),
        (os.path.join('share', package_name, 'urdf', 'control'),
         find_files('urdf/control')),
        (os.path.join('share', package_name, 'urdf', 'perception'),
         find_files('urdf/perception')),
        (os.path.join('share', package_name, 'urdf', 'scenes'),
         find_files('urdf/scenes')),
        
        # meshes
        (os.path.join('share', package_name, 'meshes'), find_files('meshes')),
        
        # config 子目录
        (os.path.join('share', package_name, 'config', 'display'),
         find_files('config/display')),
        (os.path.join('share', package_name, 'config', 'gazebo'),
         find_files('config/gazebo')),
        (os.path.join('share', package_name, 'config', 'gazebo_control'),
         find_files('config/gazebo_control')),
        (os.path.join('share', package_name, 'config', 'gazebo_eyeinhand'),
         find_files('config/gazebo_eyeinhand')),
        (os.path.join('share', package_name, 'config', 'gazebo_moveit'),
         find_files('config/gazebo_moveit')),
        
        # worlds, models
        (os.path.join('share', package_name, 'worlds'), find_files('worlds')),
        (os.path.join('share', package_name, 'models'), find_files('models')),

        # scripts
        (os.path.join('share', package_name, 'scripts'), find_files('scripts')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_to_bgr = my_arm_description.rgb_to_bgr_node:main',
            'aruco_keepalive = my_arm_description.aruco_keepalive_node:main',
            'face_servo_tracker = my_arm_description.face_servo_tracker:main',
            'face_moveit_tracker = my_arm_description.face_moveit_tracker:main',
        ],
    },
)
