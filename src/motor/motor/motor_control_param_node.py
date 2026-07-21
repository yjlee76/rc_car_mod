import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

# Int32 is used for single numbers (like a single speed or angle).
# Int32MultiArray is used for lists of numbers (like speeds for 4 different wheels).
from std_msgs.msg import Int32, Int32MultiArray

# This is the proprietary library provided by the chassis manufacturer to talk to the expansion board.
from Rosmaster_Lib import Rosmaster


class UnifiedMotorControl(Node):
    def __init__(self):
        # Initialize the ROS 2 node with the name 'motor_control'
        super().__init__("motor_control_node")

        # ---------------------------------------------------------
        # 1. HARDWARE INITIALIZATION
        # ---------------------------------------------------------
        try:
            self.bot = Rosmaster(com="/dev/myserial", delay=0.002, debug=False)
            self.bot.create_receive_threading()
            self.get_logger().info("Connected to Rosmaster on /dev/myserial")

            # Declare parameters for the center angles (default 90)
            self.declare_parameter("servo_1_center", 90)
            self.declare_parameter("servo_2_center", 15)
            self.declare_parameter("servo_3_center", 90)

            # Fetch and apply initial angles
            # s1 (steering), s2 (camera_vertical), s3 (camera_horzontal)
            s1 = self.get_parameter("servo_1_center").value
            s2 = self.get_parameter("servo_2_center").value
            s3 = self.get_parameter("servo_3_center").value

            self.bot.set_pwm_servo(1, s1)
            self.bot.set_pwm_servo(2, s2)
            self.bot.set_pwm_servo(3, s3)

            # Register the dynamic reconfigure callback
            self.add_on_set_parameters_callback(self.parameter_callback)

        except Exception as e:
            self.get_logger().error(f"Failed to connect to Rosmaster: {e}")
            return

        # ---------------------------------------------------------
        # 2. LINE FOLLOWER SUBSCRIPTIONS
        # ---------------------------------------------------------
        # These listen to the custom topics published by your line_follow_node.py.
        # They expect a single integer (Int32) for steering and speed.
        self.create_subscription(
            Int32, "/servo/steering_angle", self.steering_callback, 10
        )
        self.create_subscription(Int32, "/motor/speed", self.speed_callback, 10)

        # ---------------------------------------------------------
        # 3. LEGACY / MANUAL CONTROL SUBSCRIPTIONS
        # ---------------------------------------------------------
        # These listen to the original topics from the manufacturer.
        # They expect an array of integers (Int32MultiArray) to control specific servos or all 4 wheels.
        self.create_subscription(
            Int32MultiArray, "/servo_ctrl", self.servo_array_callback, 10
        )
        self.create_subscription(
            Int32MultiArray, "/dc_motor_ctrl", self.motor_array_callback, 10
        )

    # =========================================================
    # CALLBACK FUNCTIONS
    # These trigger automatically whenever a message arrives on their respective topics.
    # =========================================================

    def steering_callback(self, msg):
        """Called when a steering angle is received from the line follower."""
        angle = msg.data
        # Sends the angle to PWM servo #1 (usually the front steering rack).
        self.bot.set_pwm_servo(1, angle)

    def speed_callback(self, msg):
        """Called when a speed command is received from the line follower."""
        speed = msg.data
        # Applies the exact same speed to all four DC motors (Front-Left, Front-Right, Rear-Left, Rear-Right).
        self.bot.set_motor(speed, speed, speed, speed)

    def servo_array_callback(self, msg):
        """Called when a manual servo command array is received."""
        # Ensures the array contains exactly 2 items: [servo_id, angle]
        if len(msg.data) == 2:
            servo_id, angle = msg.data
            self.bot.set_pwm_servo(servo_id, angle)

    def motor_array_callback(self, msg):
        """Called when a manual motor command array is received."""
        # Ensures the array contains exactly 4 items: [speed1, speed2, speed3, speed4]
        if len(msg.data) == 4:
            s1, s2, s3, s4 = msg.data
            self.bot.set_motor(s1, s2, s3, s4)

    def parameter_callback(self, params):
        """Immediately adjusts the physical servos if their center parameter is changed."""
        for param in params:
            if param.name == "servo_1_center":
                self.bot.set_pwm_servo(1, param.value)
                self.get_logger().info(f"Servo 1 trimmed to: {param.value}")
            elif param.name == "servo_2_center":
                self.bot.set_pwm_servo(2, param.value)
                self.get_logger().info(f"Servo 2 trimmed to: {param.value}")
            elif param.name == "servo_3_center":
                self.bot.set_pwm_servo(3, param.value)
                self.get_logger().info(f"Servo 3 trimmed to: {param.value}")

        return SetParametersResult(successful=True)


def main(args=None):
    # Start the ROS 2 Python communications
    rclpy.init(args=args)
    # Instantiate our custom node class
    node = UnifiedMotorControl()

    try:
        # Keep the node running and listening for incoming topic messages infinitely
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully handle the user pressing Ctrl+C in the terminal
        node.get_logger().info("Shutting down Motor Control Node...")
    finally:
        # FAILSAFE: Always tell the hardware to stop all 4 motors before the script exits,
        # otherwise the car will keep driving forward forever.
        node.bot.set_motor(0, 0, 0, 0)
        # Destroy the node to free up memory
        node.destroy_node()
        # Shutdown ROS 2 communications
        rclpy.shutdown()


if __name__ == "__main__":
    main()
