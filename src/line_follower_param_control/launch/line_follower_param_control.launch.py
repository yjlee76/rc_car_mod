import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # LaunchDescription is simply a list of tasks or nodes that ROS 2 should start together.
    return LaunchDescription(
        [
            # 1. CAMERA NODE
            # Starts the manufacturer's camera package to capture video and publish to /image_raw.
            Node(
                package="camera",  # The name of the ROS 2 package
                executable="camera_usb_node",  # The executable name defined in its setup.py
                name="camera_usb_node",  # The custom name given to this specific running instance
                output="screen",  # Prints the node's log messages directly to the terminal
            ),
            # 2. HARDWARE CONTROL NODE
            # Starts our new unified motor package that listens for commands and talks to the serial port.
            Node(
                package="motor",
                executable="motor_control_param_node",
                name="motor_control_param_node",
                output="screen",
            ),
            # 3. VISION PROCESSING NODE
            # Starts your custom OpenCV script that subscribes to the camera, finds the yellow line,
            # and publishes speeds/angles to the motor node.
            Node(
                package="line_follower_param_control",
                executable="line_follower_param_control_node",
                name="line_follower_node_param_control_node",
                output="screen",
            ),
        ]
    )
