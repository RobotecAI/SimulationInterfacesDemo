#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "config",
            default_value="base",
            description="Dingo configuration (base, etc.)",
        )
    )

    # Initialize Arguments
    config = LaunchConfiguration("config")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindPackageShare("xacro"), "xacro"]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("dingo_description"), 
                "urdf", 
                "dingo.urdf.xacro"
            ]),
            " config:=",
            config,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint state publisher node (for visualization)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
