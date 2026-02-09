import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class WebCamViewerNode(Node):
    def __init__(self):
        super.__init__("webcamviewer")
        self.subscription = self.create_subscription(Image, 'webcam_stream', self.stream_callback, 10)
        self.cv_br = CvBridge()

    def stream_callback(self, msg):
        frame = msg.data
        frame = self.cv_br.imgmsg_to_cv2(frame)
        cv2.imshow("Display window", frame)
        cv2.waitKey(0)

        

def main(args=None):
    rclpy.init(args=args)
    node = WebCamViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
