#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from Rosmaster_Lib import Rosmaster

class MotorControlNode(Node):
    """
    Subscribes to /motor_ctrl (std_msgs/Int32MultiArray) with data = [m1, m2, m3, m4]
    and drives the corresponding motors on the Rosmaster board.

    m1-m4 speeds: -100 to 100 (percentage)
    """

    def __init__(self):
        super().__init__('motor_control')

        # ---- initialize an array to hold the offset values for all 4 motors
        self.encoder_offsets = [0, 0, 0, 0]

        # ---- parameters (overridable via ros2 run/launch) ----
        self.declare_parameter('serial_port', '/dev/myserial')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # ---- connect to the board ----
        self.bot = Rosmaster(car_type=1, com=serial_port, debug=False)
        self.bot.create_receive_threading()
        self.get_logger().info(f'Connected to Rosmaster on {serial_port} @ {baud_rate}')

        # ---- subscription ----
        self.sub_motor = self.create_subscription(
            Int32MultiArray, 'motor_ctrl', self.motor_callback, 10)

        self.get_logger().info(
            'motor_control ready. Publish to /motor_ctrl with data=[m1, m2, m3, m4] '
            'e.g. ros2 topic pub --once /motor_ctrl std_msgs/msg/Int32MultiArray "{data: [50, 50, 50, 50]}"'
        )

    def reset_encoders(self):
        """
        Call this function whenever you want to 'zero out' your encoder readings.
        It captures the current hardware counts to use as the new zero-offset.
        """
        try:
            # Fetch the raw encoder data from the board
            e1, e2, e3, e4 = self.bot.get_motor_encoder()
            
            # Save these raw values as our new baseline
            self.encoder_offsets = [e1, e2, e3, e4]
            self.get_logger().info('Encoder readings successfully reset to 0 (via software offset).')
        except Exception as e:
            self.get_logger().error(f"Failed to read encoders for reset: {e}")

    def get_zeroed_encoders(self):
        """
        Call this function to get your current encoder positions relative to your last reset.
        """
        try:
            # Read the current raw data
            e1, e2, e3, e4 = self.bot.get_motor_encoder()
            
            # Subtract the saved baseline offsets from the raw data
            real_e1 = e1 - self.encoder_offsets[0]
            real_e2 = e2 - self.encoder_offsets[1]
            real_e3 = e3 - self.encoder_offsets[2]
            real_e4 = e4 - self.encoder_offsets[3]
            
            return [real_e1, real_e2, real_e3, real_e4]
        except Exception as e:
            self.get_logger().error(f"Failed to read encoders: {e}")
            return [0, 0, 0, 0]
    
    def motor_callback(self, msg: Int32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(
                f'Expected data=[m1, m2, m3, m4], got {list(msg.data)}. Ignoring.')
            return

        m1, m2, m3, m4 = msg.data

        # Clamp speeds to safe ranges (-100 to 100)
        m1 = max(-100, min(100, m1))
        m2 = max(-100, min(100, m2))
        m3 = max(-100, min(100, m3))
        m4 = max(-100, min(100, m4))

        self.bot.set_motor(m1, m2, m3, m4)
        self.get_logger().info(f'set_motor(m1={m1}, m2={m2}, m3={m3}, m4={m4})')

    def destroy_node(self):
        try:
            # Send a stop command to all motors before shutting down
            self.bot.set_motor(0, 0, 0, 0)
            del self.bot
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()