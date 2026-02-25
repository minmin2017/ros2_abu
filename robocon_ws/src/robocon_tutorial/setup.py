from setuptools import find_packages, setup

package_name = 'robocon_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_publisher = robocon_tutorial.simple_publisher:main',
            'simple_subscriber = robocon_tutorial.simple_subscriber:main',
            'mecanum_serial_odometry = robocon_tutorial.mecanum_serial_odometry:main',
        ],
    },
)
