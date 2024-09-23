import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import logging
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv_bridge import CvBridge

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
    handlers=[logging.StreamHandler()]
)

class LEDControllerSimulator(Node):
    def __init__(self):
        super().__init__('led_controller_simulator')

        self.logger = logging.getLogger('LED_simulator')

        self.image_subscription = self.create_subscription(
            Image,
            '/booblik/sensors/led/display_image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()  # Для преобразования ROS Image в OpenCV Image
        self.pixels = (16, 22)  # Размер виртуальной матрицы светодиодов

        # Настраиваем графический вывод
        self.fig, self.ax = plt.subplots()
        self.im = self.ax.imshow(np.zeros(self.pixels), cmap='gray')  # Пустое изображение

        plt.ion()  # Включаем интерактивный режим matplotlib
        plt.show()

        # Добавляем таймер для обновления графики
        self.create_timer(0.1, self.update_display)  # Обновляем дисплей каждые 0.1 секунды

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Преобразуем изображение к размерам виртуальной светодиодной матрицы
        resized_image = cv2.resize(cv_image, self.pixels, interpolation=cv2.INTER_AREA)

        # Обновляем матрицу на графике
        self.im.set_data(cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB))  # Для отображения в RGB

    def update_display(self):
        # Обновляем графику с использованием matplotlib
        plt.draw()
        plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)

    led_controller_simulator = LEDControllerSimulator()

    try:
        rclpy.spin(led_controller_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        led_controller_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
