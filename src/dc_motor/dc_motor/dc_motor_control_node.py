#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from Rosmaster_Lib import Rosmaster


class DcMotorControlNode(Node):
    """
    Raw per-wheel DC motor control via Rosmaster_Lib.set_motor().

    Subscribes:
        /dc_motor_ctrl (std_msgs/Int32MultiArray)
            data = [speed_1, speed_2, speed_3, speed_4]
            each speed in range -100..100 (values outside are clamped)

    Publishes:
        /motor_encoder (std_msgs/Int32MultiArray)
            data = [m1, m2, m3, m4]  -- raw encoder counts, published periodically

    Note: this is low-level per-wheel control (set_motor), not chassis-level
    velocity control (set_car_motion / cmd_vel). Use this when you want direct
    control of each of the 4 wheels independently rather than X/Y/rotation.
    """

    MIN_SPEED = -100
    MAX_SPEED = 100

    def __init__(self):
        super().__init__('dc_motor_control_node')

        # ---- parameters ----
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('encoder_publish_rate', 10.0)  # Hz
        self.declare_parameter('stop_on_shutdown', True)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        encoder_rate = self.get_parameter('encoder_publish_rate').get_parameter_value().double_value
        self.stop_on_shutdown = self.get_parameter('stop_on_shutdown').get_parameter_value().bool_value

        # ---- connect to the board ----
        self.car = Rosmaster(car_type=1, com=serial_port, debug=False)
        self.car.create_receive_threading()
        self.get_logger().info(f'Connected to Rosmaster on {serial_port} @ {baud_rate}')

        # ---- subscription: raw per-wheel speed commands ----
        self.sub_motor = self.create_subscription(
            Int32MultiArray, 'dc_motor_ctrl', self.motor_callback, 10)

        # ---- publisher: encoder feedback ----
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'motor_encoder', 10)
        if encoder_rate > 0:
            period = 1.0 / encoder_rate
            self.encoder_timer = self.create_timer(period, self.encoder_timer_callback)

        self.get_logger().info(
            'dc_motor_control_node ready. Publish to /dc_motor_ctrl with '
            'data=[speed_1, speed_2, speed_3, speed_4] (-100..100), e.g. '
            'ros2 topic pub --once /dc_motor_ctrl std_msgs/msg/Int32MultiArray '
            '"{data: [30, 30, 30, 30]}"')

    def motor_callback(self, msg: Int32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'Expected data=[s1, s2, s3, s4] (4 values), got {list(msg.data)}. Ignoring.')
            return

        speeds = [max(self.MIN_SPEED, min(self.MAX_SPEED, s)) for s in msg.data]
        if list(speeds) != list(msg.data):
            self.get_logger().warn(
                f'One or more speeds out of range ({self.MIN_SPEED}..{self.MAX_SPEED}). '
                f'Clamped {list(msg.data)} -> {speeds}')

        self.car.set_motor(speeds[0], speeds[1], speeds[2], speeds[3])
        self.get_logger().info(f'set_motor{tuple(speeds)}')

    def encoder_timer_callback(self):
        m1, m2, m3, m4 = self.car.get_motor_encoder()
        msg = Int32MultiArray()
        msg.data = [int(m1), int(m2), int(m3), int(m4)]
        self.encoder_pub.publish(msg)

    def destroy_node(self):
        try:
            if self.stop_on_shutdown:
                self.car.set_motor(0, 0, 0, 0)
        except Exception:
            pass
        try:
            del self.car
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DcMotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
