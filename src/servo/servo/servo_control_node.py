#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from Rosmaster_Lib import Rosmaster


class ServoControlNode(Node):
    """
    Subscribes to /servo_ctrl (std_msgs/Int32MultiArray) with data = [servo_id, angle]
    and drives the corresponding PWM servo on the Rosmaster board.

    servo_id: 1-4  (matches Rosmaster_Lib.set_pwm_servo)
    angle:    0-180 degrees
    """

    def __init__(self):
        super().__init__('servo_control_node')

        # ---- parameters (overridable via ros2 run/launch) ----
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('min_angle', 0)
        self.declare_parameter('max_angle', 180)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.min_angle = self.get_parameter('min_angle').get_parameter_value().integer_value
        self.max_angle = self.get_parameter('max_angle').get_parameter_value().integer_value

        # ---- connect to the board ----
        self.car = Rosmaster(car_type=1, com=serial_port, debug=False)
        self.car.create_receive_threading()
        self.get_logger().info(f'Connected to Rosmaster on {serial_port} @ {baud_rate}')

        # ---- subscription ----
        self.sub_servo = self.create_subscription(
            Int32MultiArray, 'servo_ctrl', self.servo_callback, 10)

        self.get_logger().info(
            'servo_control_node ready. Publish to /servo_ctrl with data=[servo_id, angle] '
            'e.g. ros2 topic pub --once /servo_ctrl std_msgs/msg/Int32MultiArray "{data: [2, 90]}"')

    def servo_callback(self, msg: Int32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn(
                f'Expected data=[servo_id, angle], got {list(msg.data)}. Ignoring.')
            return

        servo_id, angle = msg.data

        if servo_id < 1 or servo_id > 4:
            self.get_logger().warn(f'servo_id {servo_id} out of range (1-4). Ignoring.')
            return

        if angle < self.min_angle or angle > self.max_angle:
            self.get_logger().warn(
                f'angle {angle} out of range ({self.min_angle}-{self.max_angle}). Clamping.')
            angle = max(self.min_angle, min(self.max_angle, angle))

        self.car.set_pwm_servo(servo_id, angle)
        self.get_logger().info(f'set_pwm_servo(servo_id={servo_id}, angle={angle})')

    def destroy_node(self):
        try:
            del self.car
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
