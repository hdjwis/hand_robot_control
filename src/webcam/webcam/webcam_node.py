import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebCamNode(Node):
    def __init__(self):
        super().__init__("webcam")
        self.cap = cv2.VideoCapture(1, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
        self.webcam_publisher = self.create_publisher(Image, 'webcam_stream', 10)
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cv_br = CvBridge()
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Dropped frame, retrying...")
            return
        self.webcam_publisher.publish(self.cv_br.cv2_to_imgmsg(frame))
        





def main(args=None):
    rclpy.init(args=args)
    node = WebCamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


