import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from fastai.learner import load_learner
from pathlib import Path
from PIL import Image as im



class ThumbNode(Node):
    def __init__(self):
        super().__init__("thumb")
        self.webcam_sub = self.create_subscription(Image, 'webcam_stream', self.stream_callback, 10)
        self.angle_pub = self.create_publisher(String, 'angle', 10)
        self.cv_br = CvBridge()
        self.frame = 0
        self.learn_inf = load_learner(Path() / 'export.pkl')

    def stream_callback(self, msg):
        self.frame = self.cv_br.imgmsg_to_cv2(msg)
        pil_image = im.fromarray(self.frame)
        output = self.learn_inf.predict(pil_image)
        angle = String()
        angle.data = output[0]
        self.angle_pub.publish(angle)
                

            

    


def main(args=None):
    rclpy.init(args=args)
    node = ThumbNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()