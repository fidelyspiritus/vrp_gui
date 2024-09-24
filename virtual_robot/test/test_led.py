import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class LEDImagePublisher(Node):
    def __init__(self):
        super().__init__('led_image_publisher')
        self.publisher_left = self.create_publisher(Image, '/booblik/sensors/LED/left', 10)
        self.publisher_right = self.create_publisher(Image, '/booblik/sensors/LED/right', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.publish_images)

    def publish_images(self):
        # Генерация случайных изображений для тестирования
        left_image = np.random.randint(0, 255, (8, 8, 3), dtype=np.uint8)
        right_image = np.random.randint(0, 255, (8, 8, 3), dtype=np.uint8)

        # Преобразуем изображения в формат ROS2
        left_msg = self.bridge.cv2_to_imgmsg(left_image, encoding="rgb8")
        right_msg = self.bridge.cv2_to_imgmsg(right_image, encoding="rgb8")

        # Публикуем изображения
        self.publisher_left.publish(left_msg)
        self.publisher_right.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LEDImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
