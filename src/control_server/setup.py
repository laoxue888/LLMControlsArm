from setuptools import find_packages, setup
import os

package_name = 'control_server'
share_dir = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(share_dir, 'launch'), ['launch/arm_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_trajectory_monitor_server = control_server.joint_trajectory_monitor_server:main',
            'panda_arm_mover_server = control_server.panda_arm_mover_server:main', 
            'get_end_position = control_server.get_end_position:main',
        ],
    },
)
