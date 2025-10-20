import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge

class FaceDetectorClient(Node):
    def __ini__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector,'/face_detect')
        self.bridge = CvBridge()
        self.test1_image_path = get_package_share_directory(
            'demo_python_service')+'/resource/test1.jpg'
        self.image = cv2.imread(self.test1_image_path)

    def send_request(self):
        # TODO: 发送请求并处理结果
        return

    def show_face_locations(self,response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image,(left,top),(right,bottom),(255,0,0),2)
        cv2.imshow('Face Detection Result',self.image)
        cv2.waitKey(0)

def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetectorClient()
    face_detect_client.send_request()
    rclpy.shutdown()
