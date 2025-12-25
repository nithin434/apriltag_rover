#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    pkg_name = 'apriltag_docking'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'camera_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    
    # Process URDF
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Gazebo server (headless) - use simple empty world
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mecanum_robot',
                   '-topic', '/robot_description',
                   '-x', '0', '-y', '0', '-z', '0.15'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Mission controller
    mission_controller = Node(
        package=pkg_name,
        executable='mission_controller',
        output='screen'
    )
    
    # AprilTag follower
    apriltag_follower = Node(
        package=pkg_name,
        executable='apriltag_follower',
        output='screen'
    )
    
    # Synthetic camera for Foxglove visualization
    synthetic_camera = Node(
        package=pkg_name,
        executable='synthetic_camera',
        output='screen'
    )
    
    # ROS bag recording with camera
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '/camera/image_raw',
             '/camera/camera_info', 
             '/cmd_vel',
             '/odom',
             '/target_tag',
             '/tag_reached',
             '/lift_position',
             '/tf',
             '/tf_static',
             '-o', '/root/docking_ws/camera_mission_' + '$(date +%Y%m%d_%H%M%S)'],
        output='screen',
        shell=True
    )
    
    return LaunchDescription([
        gzserver,
        robot_state_pub,
        spawn_robot,
        mission_controller,
        apriltag_follower,
        synthetic_camera,
        rosbag_record,
    ])