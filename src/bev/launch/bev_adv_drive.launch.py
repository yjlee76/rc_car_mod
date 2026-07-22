from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 1. Start the Camera
            Node(
                package="camera",
                executable="camera_usb_node",
                name="camera_usb_node",
                output="screen",
            ),
            # 2. Start the Hardware Controller
            Node(
                package="motor",
                executable="motor_control_param_node",
                name="motor_control_param_node",
                output="screen",
            ),
            # 3. Start the Advanced Vision Pipeline
            Node(
                package="bev",
                executable="bev_adv_node",  # Targeting your new advanced node
                name="bev_adv_node",
                output="screen",
            ),
        ]
    )
