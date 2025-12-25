from setuptools import setup
import os

package_name = 'apriltag_docking'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/apriltag_docking']),
        ('share/apriltag_docking', ['package.xml']),
        ('share/apriltag_docking/launch',
            ['launch/docking_demo.launch.py',
             'launch/docking_headless.launch.py']),
        ('share/apriltag_docking/worlds',
            ['worlds/docking_world.world']),
        ('share/apriltag_docking/urdf',
            ['urdf/mecanum_robot.urdf.xacro',
             'urdf/rack.urdf.xacro']),
        ('share/apriltag_docking/config',
            ['config/apriltag_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='AprilTag docking demo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_controller = apriltag_docking.mission_controller:main',
            'apriltag_follower = apriltag_docking.apriltag_follower:main',
            'synthetic_camera = apriltag_docking.synthetic_camera:main',
        ],
    },
)
