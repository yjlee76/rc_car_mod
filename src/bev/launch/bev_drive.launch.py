from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 1. Start the camera node
            Node(
                package="camera",
                executable="camera_usb_node",
                name="camera_usb_node",
                output="screen",
            ),
            # 2. Start the motor node
            Node(
                package="motor",
                executable="motor_control_param_node",
                name="motor_control_control_node",
                output="screen",
            ),
            # 3. Start your new Advanced Vision Pipeline
            Node(
                package="bev", executable="bev_node", name="bev_node", output="screen"
            ),
        ]
    )
