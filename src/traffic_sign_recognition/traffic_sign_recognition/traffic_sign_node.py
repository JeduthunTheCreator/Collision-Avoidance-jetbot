import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
import numpy as np
import cv2

class TrafficSignNode(Node):
    def __init__(self):
        super().__init__('traffic_sign_recognition_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.model = tf.keras.models.load_model('home/ros2_ws/src/traffic_sign_recognition/traffic_sign_recognition.h5')

    def image_callback(self, msg):
        try:
            # Convert ROS image to CV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Preprocess image
            processed_image = self.preprocess_image(cv_image)
            # Make prediction
            prediction = self.model.predict(processed_image)
            sign_class = np.argmax(prediction, axis=1)[0]
            self.get_logger().info('Detected Traffic Sign: {}'.format(sign_class))
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: {}'.format(str(e)))

    def preprocess_image(self, image):
        image = cv2.resize(image, (32, 32))  # Resize as per the model's requirement
        image = np.expand_dims(image, axis=0)  # Add batch dimension
        image = image / 255.0  # Normalize the image
        return image

def main(args=None):
    rclpy.init(args=args)
    traffic_sign_node = TrafficSignNode()
    rclpy.spin(traffic_sign_node)
    traffic_sign_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

