import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from random import uniform
import logging

logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
        handlers=[logging.StreamHandler()]
)

class GPSNode(Node):
    def __init__ (self, name = 'GPS'):
        super().__init__(name)
        self.logger = logging.getLogger('GPS_simulation')

        self.nav_ = self.create_publisher(
            NavSatFix,
            '/booblik/sensors/gps/navsat/fix',
            10
            )
        
        self.odometry_ = self.create_publisher(
            Odometry,
            '/booblik/sensors/position/ground_truth_odometry',
            10
        )

        self.latitude = 60.012575   # Ольгинский пруд для тестов
        self.longitude = 30.357604 

        self.timer_ = self.create_timer(1.0, self.publish_gps)

    def publish_gps(self):
        msg = NavSatFix()
    
        # Имитируем случайное смещение
        self.latitude += uniform(-0.0001, 0.0001)
        self.longitude += uniform(-0.0001, 0.0001)

        msg.latitude = self.latitude
        msg.longitude = self.longitude
        self.nav_.publish(msg)

        self.logger.info(f'Simulated GPS: {self.latitude=:.6f}, {self.longitude=:.6f}')

    def publish_odometry(self):
        ...

def main(args=None):
    rclpy.init(args=args)
    task = GPSNode()
    rclpy.spin(task)
    rclpy.shutdown

if __name__ == '__main__':
    main()