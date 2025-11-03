import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3e_hande_moveit_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')) + glob(os.path.join(package_name, 'launch', '*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clinton Enwerem',
    maintainer_email='enwerem@terpmail.umd.edu',
    description='The ur3e_hande_moveit_scripts package.',
    license='BSD',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ur3_joint_goal_node = ur3e_hande_moveit_scripts.scripts.ur3_joint_goal:main',
            'ur3_pose_goal_node = ur3e_hande_moveit_scripts.scripts.ur3_pose_goal:main',
            'ur3_moveit_config_node = ur3e_hande_moveit_scripts.scripts.ur3_moveit_config:main',
            'ur3_moveit_scene_node = ur3e_hande_moveit_scripts.scripts.ur3_moveit_scene:main',
            'hande_command_node = ur3e_hande_moveit_scripts.scripts.hande_command:main',
        ],
    },
)
