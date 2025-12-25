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
    world_file = os.path.join(pkg_share, 'worlds', 'docking_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    
    # Process URDF
    robot_description = xacro.process_file(urdf_file).toxml()
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
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
    
    # Lift controller
    lift_controller = Node(
        package='gazebo_ros',
        executable='joint_position_controller.py',
        parameters=[{
            'joint_name': 'lift_joint',
            'topic_name': '/lift_position'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_pub,
        spawn_robot,
        mission_controller,
        apriltag_follower,
        # lift_controller,  # Optional if gazebo plugin handles it
    ])
