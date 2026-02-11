import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import mediapipe as mp
import math
from std_msgs.msg import Float32



class ThumbNode(Node):
    def __init__(self):
        super().__init__("thumb")
        self.absolute_model_path = "/Users/adarshraju/Desktop/Projects/hand_robot_control/hand_landmarker.task"
        self.webcam_sub = self.create_subscription(Image, 'webcam_stream', self.stream_callback, 10)
        self.angle_pub = self.create_publisher(Float32, 'angle', 10)
        self.cv_br = CvBridge()
        self.frame = 0
        self.BaseOptions = mp.tasks.BaseOptions
        self.HandLandmarker = mp.tasks.vision.HandLandmarker
        self.HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        # self.HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
        self.VisionRunningMode = mp.tasks.vision.RunningMode

    # def landmark_callback(self, result: mp.tasks.vision.HandLandmarkerResult, output_image: mp.Image, time_stamp_ms: int):
    #     if result.hand_landmarks != []:
    #         print(result.hand_landmarks)
    #     # print(result.hand_landmarks[4])

    def stream_callback(self, msg):
        self.frame = self.cv_br.imgmsg_to_cv2(msg)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.frame)
        options = self.HandLandmarkerOptions(
        base_options=self.BaseOptions(model_asset_path=self.absolute_model_path),
        running_mode=self.VisionRunningMode.IMAGE,
        min_hand_detection_confidence = 0.1)
        with self.HandLandmarker.create_from_options(options) as landmarker:
            result = landmarker.detect(mp_image)
            if len(result.hand_landmarks) > 0:
                x_change = result.hand_landmarks[0][1].x - result.hand_landmarks[0][4].x
                y_change = result.hand_landmarks[0][1].y - result.hand_landmarks[0][4].y
                angle = Float32()
                angle.data = (180*math.atan(y_change/x_change)/math.pi) % 360
                
                self.angle_pub.publish(angle)
                

            

    


def main(args=None):
    rclpy.init(args=args)
    node = ThumbNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()