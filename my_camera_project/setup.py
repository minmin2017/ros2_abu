from setuptools import find_packages, setup

package_name = 'my_camera_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minmin',
    maintainer_email='minmin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_driver = my_camera_project.camera_driver:main',
            'camera_sub = my_camera_project.camera:main',
            'yolo_detector = my_camera_project.yolo_detector:main',
        ],
    },
)
