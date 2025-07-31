import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from torchvision import models, transforms
from std_msgs.msg import String  

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.publisher = self.create_publisher(String, '/detection_alerts', 10)
        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.handle_image, 10)
        self.bridge = CvBridge()
        self.model = models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        self.model.eval()
        self.transform = transforms.Compose([
            transforms.ToTensor()
        ])

    def handle_image(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        input_tensor = self.transform(cv_image)
        predictions = self.model([input_tensor])  # Assuming the model returns a dictionary of predictions
        
        # Process predictions and publish actionable alerts
        for prediction in predictions:
            if prediction['labels'] == 1: 
                self.publisher.publish(String(data="Stop"))

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

