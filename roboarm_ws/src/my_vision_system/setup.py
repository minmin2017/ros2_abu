from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_vision_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
        (os.path.join('share', package_name, 'models_upgrade'), glob('models_upgrade/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='poomjai',
    maintainer_email='poomjai@todo.todo',
    description='ROS2 YOLO Docking System',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = my_vision_system.yolo_node:main',
            'yolo_docking_node = my_vision_system.yolo_docking_node:main',
            'yolo_select_node = my_vision_system.yolo_select_node:main',
            'yolo_real_cam_debug = my_vision_system.yolo_real_cam_debug:main'
        ],
    },
)
