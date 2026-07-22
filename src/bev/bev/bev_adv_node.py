import cv2  # OpenCV library for image processing and computer vision functions
import numpy as np  # NumPy library for matrix operations and linear algebra
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base ROS 2 Node class
from sensor_msgs.msg import Image  # Message type for receiving camera video frames
from std_msgs.msg import Int32  # Message type for sending steering and speed commands
from cv_bridge import (
    CvBridge,
)  # Bridge to convert ROS 2 Image messages to OpenCV format
from rcl_interfaces.msg import (
    SetParametersResult,
)  # Return type required for dynamic parameter callbacks


class BevAdvNode(Node):
    """
    An advanced ROS 2 autonomous driving node utilizing Bird's-Eye View (BEV) perspective transformation,
    dual-channel (HLS color + Sobel X gradient) thresholding, a single-lane sliding window algorithm,
    and a combined Feedforward (Curvature) + Cross-Track Error (CTE) steering controller.
    """

    def __init__(self):
        # Initialize the node under the registered ROS 2 name 'bev_adv_node'
        super().__init__("bev_adv_node")

        # ---------------------------------------------------------
        # 1. ROS 2 INFRASTRUCTURE SETUP
        # ---------------------------------------------------------
        # CvBridge handles conversion between sensor_msgs/msg/Image and OpenCV numpy arrays
        self.bridge = CvBridge()

        # Subscribe to raw camera video topic
        self.subscription = self.create_subscription(
            Image, "/image_raw", self.image_callback, 10
        )

        # Publishers for motor velocity and servo steering commands
        self.steering_pub = self.create_publisher(Int32, "/servo/steering_angle", 10)
        self.motor_pub = self.create_publisher(Int32, "/motor/speed", 10)

        # ---------------------------------------------------------
        # 2. DYNAMIC PARAMETER SERVER
        # ---------------------------------------------------------
        # Declare parameters so they can be modified live via rqt_reconfigure or CLI
        self.declare_parameter("base_speed", 40)  # Constant forward motor power
        self.declare_parameter(
            "center_steering", 90
        )  # Servo angle for straight driving (degrees)
        self.declare_parameter(
            "lane_target_ratio", 0.25
        )  # Target position of center line (25% from left screen edge)

        # Physical control gains:
        self.declare_parameter(
            "kp_cte", 150.0
        )  # Proportional Cross-Track Gain (steering degrees per meter of drift)
        self.declare_parameter(
            "kf", 15.0
        )  # Feedforward Gain (steering degrees per unit of path curvature)

        # Assign parameter values to internal instance variables
        self.base_speed = self.get_parameter("base_speed").value
        self.center_steering = self.get_parameter("center_steering").value
        self.lane_target_ratio = self.get_parameter("lane_target_ratio").value
        self.kp_cte = self.get_parameter("kp_cte").value
        self.kf = self.get_parameter("kf").value

        # Register callback for dynamic parameter reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ---------------------------------------------------------
        # 3. CAMERA CALIBRATION MATRICES (INTRINSICS)
        # ---------------------------------------------------------
        # Camera Matrix (mtx): Specifies focal length (fx, fy) and principal point (cx, cy)
        self.camera_matrix = np.array(
            [[500.123, 0.0, 320.5], [0.0, 500.456, 240.2], [0.0, 0.0, 1.0]]
        )
        # Distortion Coefficients (dist): Corrects radial and tangential lens distortion
        self.dist_coeffs = np.array([[-0.123, 0.045, -0.001, 0.002, -0.003]])

        # ---------------------------------------------------------
        # 4. REAL-WORLD METRIC CONVERSIONS
        # ---------------------------------------------------------
        # Scaling factors mapping pixel dimensions in BEV space to physical meters in world space
        self.ym_per_pix = (
            0.3 / 480
        )  # Estimated meters per pixel along the vertical axis (Y)
        self.xm_per_pix = (
            0.2 / 640
        )  # Estimated meters per pixel along the horizontal axis (X)

    def parameter_callback(self, params):
        """
        Callback triggered dynamically when ROS 2 parameters are updated at runtime.
        """
        for param in params:
            if param.name == "base_speed":
                self.base_speed = param.value
            elif param.name == "center_steering":
                self.center_steering = param.value
            elif param.name == "lane_target_ratio":
                self.lane_target_ratio = param.value
            elif param.name == "kp_cte":
                self.kp_cte = param.value
            elif param.name == "kf":
                self.kf = param.value
        return SetParametersResult(successful=True)

    def thresholding(self, img):
        """
        Applies HLS color filtering and Sobel horizontal gradient edge detection to isolate lane lines.
        """
        # Convert frame from BGR to HLS color space
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        # --- Channel 1: Color Thresholding (Saturation) ---
        s_channel = hls[:, :, 2]
        s_binary = np.zeros_like(s_channel)
        # Isolate highly saturated yellow lane lines
        s_binary[(s_channel >= 120) & (s_channel <= 255)] = 1

        # --- Channel 2: Gradient Thresholding (Sobel X) ---
        l_channel = hls[:, :, 1]
        # Calculate vertical edge derivatives along the horizontal axis
        sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(sobelx)
        scaled_sobel = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))
        sxbinary = np.zeros_like(scaled_sobel)
        sxbinary[(scaled_sobel >= 20) & (scaled_sobel <= 100)] = 1

        # --- Merge Threshold Masks ---
        combined_binary = np.zeros_like(sxbinary)
        # Bitwise OR: retain pixel if either color or gradient criteria is met
        combined_binary[(s_binary == 1) | (sxbinary == 1)] = 255
        return combined_binary

    def sliding_window(self, binary_warped):
        """
        Identifies active lane pixels using a histogram peak search restricted to the left half
        (center line) and tracks curvature upwards using stacked bounding boxes.
        """
        # Compute vertical pixel intensity sum across the lower half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0] // 2 :, :], axis=0)
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))

        # --- HISTOGRAM SPLITTING ---
        # Search for highest peak ONLY within the left half of the image
        midpoint = int(histogram.shape[0] // 2)
        current_x = np.argmax(histogram[:midpoint])

        # Hyperparameters for window sliding
        nwindows = 9  # Number of vertical search windows
        window_height = int(
            binary_warped.shape[0] / nwindows
        )  # Pixel height per window
        margin = 50  # Half-width search region around center
        minpix = 40  # Threshold pixels required to shift window center

        # Extract coordinates of all active non-zero pixels
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        lane_inds = []

        # Iterate through windows bottom-to-top
        for window in range(nwindows):
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_x_low = current_x - margin
            win_x_high = current_x + margin

            # Render green bounding box for debug visualization
            cv2.rectangle(
                out_img,
                (win_x_low, win_y_low),
                (win_x_high, win_y_high),
                (0, 255, 0),
                2,
            )

            # Find indices of active pixels within current bounding box bounds
            good_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_x_low)
                & (nonzerox < win_x_high)
            ).nonzero()[0]

            lane_inds.append(good_inds)

            # Recenter next window on mean position if sufficient pixels are captured
            if len(good_inds) > minpix:
                current_x = int(np.mean(nonzerox[good_inds]))

        # Flatten list of pixel indices
        lane_inds = np.concatenate(lane_inds)
        x_coords = nonzerox[lane_inds]
        y_coords = nonzeroy[lane_inds]

        return x_coords, y_coords, out_img

    def image_callback(self, msg):
        """
        Main execution loop triggered upon arrival of each camera frame.
        """
        # Convert ROS image message to OpenCV BGR image
        raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Correct optical lens distortion using intrinsic matrices
        cv_image = cv2.undistort(
            raw_image, self.camera_matrix, self.dist_coeffs, None, self.camera_matrix
        )
        h, w = cv_image.shape[:2]

        # Define perspective transformation source trapezoid and destination rectangle
        src_pts = np.float32([[w * 0.2, h * 0.6], [w * 0.8, h * 0.6], [0, h], [w, h]])
        dst_pts = np.float32([[0, 0], [w, 0], [0, h], [w, h]])

        # Warp perspective into top-down Bird's-Eye View
        warp_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        birdseye = cv2.warpPerspective(cv_image, warp_matrix, (w, h))

        # Perform thresholding and extract lane pixel coordinates
        binary_warped = self.thresholding(birdseye)
        x_coords, y_coords, sliding_window_img = self.sliding_window(binary_warped)

        if len(x_coords) > 0:
            # ---------------------------------------------------------
            # 1. PIXEL POLYNOMIAL FIT (Visual Debugging)
            # ---------------------------------------------------------
            # Fit a 2nd-order polynomial in pixel space: x = A*y^2 + B*y + C
            fit_pixels = np.polyfit(y_coords, x_coords, 2)
            ploty = np.linspace(0, h - 1, h)
            fitx = fit_pixels[0] * ploty**2 + fit_pixels[1] * ploty + fit_pixels[2]

            # Render polynomial overlay in red on visual output image
            pts = np.array([np.transpose(np.vstack([fitx, ploty]))], np.int32)
            cv2.polylines(sliding_window_img, pts, False, (0, 0, 255), 4)

            # ---------------------------------------------------------
            # 2. PHYSICAL KINEMATICS & POLYNOMIAL FIT (Meters)
            # ---------------------------------------------------------
            # Fit polynomial converting pixel coordinates to physical meters
            fit_cr = np.polyfit(
                y_coords * self.ym_per_pix, x_coords * self.xm_per_pix, 2
            )
            y_eval_m = np.max(ploty) * self.ym_per_pix

            # Compute signed curvature (kappa): K = y'' / (1 + (y')^2)^(3/2)
            numerator = 2 * fit_cr[0]
            denominator = (1 + (2 * fit_cr[0] * y_eval_m + fit_cr[1]) ** 2) ** 1.5
            curvature = numerator / denominator

            # Calculate Cross-Track Error (CTE) in physical meters
            y_eval_pix = np.max(ploty)
            lane_bottom_x = (
                fit_pixels[0] * y_eval_pix**2
                + fit_pixels[1] * y_eval_pix
                + fit_pixels[2]
            )
            target_x = w * self.lane_target_ratio

            error_pixels = lane_bottom_x - target_x
            cte_meters = error_pixels * self.xm_per_pix

            # Log real-time physical metrics
            self.get_logger().info(
                f"Curvature: {curvature:.3f} | CTE: {cte_meters:.3f}m"
            )

            # ---------------------------------------------------------
            # 3. ADVANCED STEERING CONTROL (FEEDFORWARD + PROPORTIONAL CTE)
            # ---------------------------------------------------------
            # Feedforward term: anticipates steering demand based on road curvature
            steer_ff = curvature * self.kf

            # Feedback term: corrects physical lateral drift from target path
            steer_cte = cte_meters * self.kp_cte

            # Sum terms to form total steering adjustment
            steering_adjustment = int(steer_ff + steer_cte)

            # Compute final steering angle and clamp within physical hardware limits
            final_angle = self.center_steering + steering_adjustment
            final_angle = max(45, min(135, final_angle))

            # Publish commands
            servo_msg = Int32()
            servo_msg.data = final_angle
            self.steering_pub.publish(servo_msg)

            motor_msg = Int32()
            motor_msg.data = self.base_speed
            self.motor_pub.publish(motor_msg)

            # Draw target lane position line (cyan) on output display
            cv2.line(
                sliding_window_img,
                (int(target_x), 0),
                (int(target_x), h),
                (255, 255, 0),
                2,
            )

        else:
            # Emergency stop trigger if lane boundaries are lost
            self.get_logger().warn("Track lost! Halting.")
            stop_msg = Int32()
            stop_msg.data = 0
            self.motor_pub.publish(stop_msg)

        # Render OpenCV debugging views
        cv2.imshow("1. Advanced Thresholding", binary_warped)
        cv2.imshow("2. Adv Control & Polyfit", sliding_window_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BevAdvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up OpenCV windows and terminate ROS 2 node context
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
