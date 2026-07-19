import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from jetcam.usb_camera import USBCamera

class CameraUSBNode(Node):
    def __init__(self):
        super().__init__('camera_usb_node')
        
        # Create the ROS2 publisher
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        
        # Initialize JetCam for a USB camera (/dev/video0 by default)
        # Adjust capture_device to match your /dev/videoX index if needed
        self.camera = USBCamera(width=640, height=480, capture_width=640, capture_height=480, capture_device=0)
        
        # Use a ROS timer to pull frames synchronously instead of traitlets
        timer_period = 1.0 / 30.0  # 30 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera_usb_node started! Publishing to /image_raw')

    def timer_callback(self):
        try:
            # Read a frame (jetcam returns a BGR8 numpy array)
            cv_image = self.camera.read()
            
            if cv_image is not None:
                # Convert the OpenCV image to a ROS2 Image message
                msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to capture or publish image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraUSBNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
