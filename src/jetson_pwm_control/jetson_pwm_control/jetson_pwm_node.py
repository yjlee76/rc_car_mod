import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import os


class JetsonPWMNode(Node):
    def __init__(self):
        super().__init__("jetson_pwm_node")

        # Hardware addresses mapped to pins
        self.pin_map = {"15": "3280000", "32": "32e0000", "33": "32c0000"}
        self.pwm_paths = {}

        # Dynamically locate the pwmchip paths
        for pin, addr in self.pin_map.items():
            path = self.get_pwm_path(addr)
            if path:
                self.pwm_paths[pin] = path
                self.get_logger().info(f"Pin {pin} mapped to {path}")
            else:
                self.get_logger().error(f"Could not find active pwmchip for Pin {pin}")

        if self.pwm_paths:
            self.get_logger().info(
                'Jetson pwm control has been started. Publish to /pwm/pinXX std_msgs/msg/Int32 "{data: duty_cycle_in_ns}. '
                'e.g. ros2 topic pub --once /pwm/pin15 std_msgs/msg/Int32 "{data: 1500000}"'
            )

        # Create subscribers for each pin
        self.sub15 = self.create_subscription(
            Int32, "pwm/pin15", lambda msg: self.write_pwm("15", msg.data), 10
        )
        self.sub32 = self.create_subscription(
            Int32, "pwm/pin32", lambda msg: self.write_pwm("32", msg.data), 10
        )
        self.sub33 = self.create_subscription(
            Int32, "pwm/pin33", lambda msg: self.write_pwm("33", msg.data), 10
        )

    def get_pwm_path(self, base_address):
        """Locates the pwm0 directory based on the hardware address symlink."""
        try:
            for chip in os.listdir("/sys/class/pwm"):
                if chip.startswith("pwmchip"):
                    device_path = os.path.join("/sys/class/pwm", chip, "device")
                    if os.path.exists(device_path):
                        target = os.readlink(device_path)
                        if base_address in target:
                            return f"/sys/class/pwm/{chip}/pwm0"
        except Exception as e:
            self.get_logger().error(f"Error finding PWM path: {e}")
        return None

    def write_pwm(self, pin, duty_cycle_ns):
        """Writes the requested nanosecond pulse to the sysfs file."""
        if pin not in self.pwm_paths:
            return

        duty_file = os.path.join(self.pwm_paths[pin], "duty_cycle")
        try:
            with open(duty_file, "w") as f:
                f.write(str(duty_cycle_ns))
            self.get_logger().debug(f"Pin {pin} updated to {duty_cycle_ns} ns")
        except PermissionError:
            self.get_logger().error(
                f"Permission denied writing to {duty_file}. Did you run the hardware setup script?"
            )
        except Exception as e:
            self.get_logger().error(f"Error writing to Pin {pin}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JetsonPWMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
