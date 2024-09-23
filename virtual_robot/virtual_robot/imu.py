import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from math import radians
from random import uniform
import logging

logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
        handlers=[logging.StreamHandler()]
    )

class ImuNode(Node):
    def __init__(self, name='IMU_simulator'):
        super().__init__(name)
        self.logger = logging.getLogger('Compas_simulator') # Создание логгера для данных 

        self.imu_ = self.create_publisher(
            Vector3,
            '/booblik/sensors/imu/imu/euler',
            10
        )
        self.timer = self.create_timer(1.0, self.publish_imu)

        self.yaw = 0.0 

    def publish_imu(self):
        msg = Vector3()
        self.yaw += uniform(-4.0, 4.0)  #   Текущая версия для проверки роботоспособности системы в целом

        msg.x = 0.0
        msg.y = radians(self.yaw)
        msg.z = 0.0 

        self.imu_.publish(msg)
        self.logger.info(f'Simulated IMU: {self.yaw=:.2f} degrees')


def main(args=None):
    rclpy.init(args=args)
    task = ImuNode()
    rclpy.spin(task)
    rclpy.shutdown

if __name__ == '__main__':
    main()
