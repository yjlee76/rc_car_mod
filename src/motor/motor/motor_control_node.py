import rclpy
from rclpy.node import Node

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

            # --- NEW: INITIALIZE SERVOS ---
            # Set servos 1 (steering), 2 (camera vertical), and 3 (camera horizontal) to their center position (90 degrees) on startup.
            # You can change '90' to whatever default angle your specific hardware requires.
            self.bot.set_pwm_servo(1, 90)
            self.bot.set_pwm_servo(2, 15)
            self.bot.set_pwm_servo(3, 90)
            self.get_logger().info("Servos 1, 2, and 3 initialized to 90 degrees.")

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
