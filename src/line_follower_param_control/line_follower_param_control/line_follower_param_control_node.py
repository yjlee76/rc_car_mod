import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult


class LineFollower(Node):
    def __init__(self):
        # Initialize the ROS 2 node with the name 'line_follower'
        super().__init__("line_follower_param_control_node")

        # ---------------------------------------------------------
        # 1. SETUP ROS 2 COMMUNICATIONS
        # ---------------------------------------------------------

        # Subscribe to the camera feed.
        # Update '/camera/image_raw' to match the actual topic published by your jetcam or usb_camera node.
        self.subscription = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10  # QoS profile depth
        )

        # Create publishers to send commands to the hardware abstraction nodes.
        # Update these topic names to match what your dc_motor and servo nodes are listening to.
        self.steering_pub = self.create_publisher(Int32, "/servo/steering_angle", 10)
        self.motor_pub = self.create_publisher(Int32, "/motor/speed", 10)

        # CvBridge is a ROS utility that translates ROS Image messages into OpenCV format (NumPy arrays).
        self.bridge = CvBridge()

        # Publisher to broadcast the processed mask image
        self.mask_pub = self.create_publisher(Image, "/vision/mask", 10)

        # ---------------------------------------------------------
        # 2. SETUP CONTROL PARAMETERS
        # ---------------------------------------------------------

        # Proportional control gain (Kp).
        # This determines how aggressively the car steers to correct an error.
        # Start small and increase it until the car tracks smoothly without oscillating.
        self.declare_parameter("kp", 0.2)

        # The default forward speed of the car when it sees the line.
        self.declare_parameter("base_speed", 40)

        # Read the initial values into class variables
        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.base_speed = (
            self.get_parameter("base_speed").get_parameter_value().integer_value
        )

        # The PWM or angle value that represents "straight ahead" for your steering servo.
        self.center_steering = 90

        # Register the callback to listen for live parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(
            "Vision Line Follower initialized. Waiting for camera feed..."
        )

    def parameter_callback(self, params):
        """This function triggers automatically whenever a parameter is changed dynamically."""
        for param in params:
            if param.name == "kp":
                self.kp = param.value
                self.get_logger().info(f"Updated kp to: {self.kp}")
            elif param.name == "base_speed":
                self.base_speed = param.value
                self.get_logger().info(f"Updated base_speed to: {self.base_speed}")

        # Return success so ROS 2 knows the update was accepted
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        """
        This function is called automatically every time a new image frame is received from the camera.
        It contains the entire vision processing and motor control pipeline.
        """

        # ---------------------------------------------------------
        # STEP 1: CONVERT IMAGE
        # ---------------------------------------------------------
        # Convert the incoming ROS 2 message to an OpenCV BGR (Blue, Green, Red) image matrix.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Get the height and width of the image.
        height, width, _ = cv_image.shape

        # ---------------------------------------------------------
        # STEP 2: REGION OF INTEREST (ROI)
        # ---------------------------------------------------------
        # We don't want to process the whole image because background objects (like yellow posters on a wall)
        # might confuse the car. We crop the image to only look at the bottom 40% (the road).
        roi_start_height = int(height * 0.6)
        roi = cv_image[roi_start_height:height, 0:width]

        # ---------------------------------------------------------
        # STEP 3: COLOR CONVERSION (BGR to HSV)
        # ---------------------------------------------------------
        # Convert the cropped road image from BGR to HSV (Hue, Saturation, Value).
        # HSV is much better for color tracking than BGR because it separates color (Hue) from lighting (Value),
        # making it robust against shadows and uneven room lighting.
        hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # ---------------------------------------------------------
        # STEP 4: COLOR THRESHOLDING (MASKING)
        # ---------------------------------------------------------
        # Define the lower and upper bounds of the color yellow in the HSV space.
        # Note: You may need to tune these values based on your specific lighting and tape color.
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Create a binary mask. This turns every pixel that falls inside the yellow range into pure WHITE (255),
        # and every other pixel into pure BLACK (0).
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # ---------------------------------------------------------
        # STEP 5: FIND THE CENTER OF THE LINE
        # ---------------------------------------------------------
        # Calculate the "moments" of the binary mask.
        # In computer vision, moments are statistical calculations of pixel intensities that help us find the
        # area and center of mass (centroid) of an object (our white pixels).
        M = cv2.moments(mask)

        # Check if the area ('m00') is greater than zero to ensure we actually see the yellow line.
        if M["m00"] > 0:
            # Calculate the X and Y coordinates of the centroid of the yellow line.
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # ---------------------------------------------------------
            # STEP 6: CALCULATE TRACKING ERROR
            # ---------------------------------------------------------
            # The error is the horizontal distance in pixels between the center of the camera (where we want the line)
            # and the actual center of the line (cx).
            screen_center = width // 2
            error = cx - screen_center

            # ---------------------------------------------------------
            # STEP 7: PROPORTIONAL (P) CONTROL FOR STEERING
            # ---------------------------------------------------------
            # Multiply the error by our proportional gain (kp) to calculate the steering adjustment.
            # If the error is large (line is far to the side), it steers sharply.
            # If the error is small (line is near the middle), it steers gently.
            steering_adjustment = error * self.kp

            # Apply the adjustment to the center steering position.
            # Depending on your servo hardware, you might need to subtract instead of add to steer the correct way.
            steering_command = int(self.center_steering + steering_adjustment)

            # Clamp the steering command so we don't send illegal values that could break the physical servo.
            steering_command = max(45, min(135, steering_command))

            # ---------------------------------------------------------
            # STEP 8: PUBLISH COMMANDS
            # ---------------------------------------------------------
            # Create the message payload and publish the steering command
            steer_msg = Int32()
            steer_msg.data = steering_command
            self.steering_pub.publish(steer_msg)

            # Create the message payload and publish the forward speed command
            motor_msg = Int32()
            motor_msg.data = self.base_speed
            self.motor_pub.publish(motor_msg)

        else:
            # ---------------------------------------------------------
            # STEP 9: FAILSAFE (LINE LOST)
            # ---------------------------------------------------------
            # If no yellow pixels are found in the mask, the area ('m00') is 0.
            # We must stop the car to prevent it from crashing or driving off.
            self.get_logger().warn("Yellow line not detected! Stopping car.")

            stop_msg = Int32()
            stop_msg.data = 0
            self.motor_pub.publish(stop_msg)

        # ---------------------------------------------------------
        # STEP 10: DISPLAY IMAGES
        # ---------------------------------------------------------
        # Show the raw, unprocessed camera feed
        cv2.imshow("Raw Camera Feed", cv_image)

        # Show the cropped region of interest (the bottom 40%)
        cv2.imshow("Region of Interest", roi)

        # Show the binary mask (what the computer actually 'sees' as yellow)
        cv2.imshow("Yellow Mask", mask)

        # Required to tell OpenCV to pause for 1 millisecond and draw the windows
        cv2.waitKey(1)

        # ROS 2 way for visualization (rqt_image_view)
        # Convert the OpenCV binary mask (mono8 encoding) back to a ROS 2 message
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")

        # Publish the message to the network
        self.mask_pub.publish(mask_msg)


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Instantiate the node
    node = LineFollower()

    try:
        # Keep the node running, actively listening for incoming camera messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Handle user pressing Ctrl+C gracefully
        node.get_logger().info("Shutting down Vision Line Follower...")
    finally:
        # Before shutting down, ensure the car is stopped
        stop_msg = Int32()
        stop_msg.data = 0
        node.motor_pub.publish(stop_msg)

        # Clean up ROS 2 resources
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
