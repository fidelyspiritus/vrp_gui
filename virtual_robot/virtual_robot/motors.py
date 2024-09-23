from std_msgs.msg import Float64
import rclpy
from rclpy.node import Node
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(name)s] [%(levelname)-5.5s] %(message)s',
    handlers=[logging.StreamHandler()]
)

class MotorSimulator(Node):

    def __init__(self):
        super().__init__('motor_simulator')
        self.logger = logging.getLogger('Motors_simulated')

        self.motors = {
            'left': '/booblik/thrusters/left/thrust',
            'right': '/booblik/thrusters/right/thrust',
            'back': '/booblik/thrusters/back/thrust'
        }

        # Подписываемся на команды для каждого двигателя
        for motor_name, topic in self.motors.items():
            setattr(self, f'{motor_name}_thrust',0) # Инициализация тяги для каждого двигателя 
            self.create_subscription(Float64, topic, lambda msg, name=motor_name: self.motor_callback(msg, name), 10)

        # Публикация данных о состоянии двигателей каждую секунду
        self.timer = self.create_timer(1.0, self.publish_status)

    def motor_callback(self, msg, motor_name):
        setattr(self, f"{motor_name}_thrust", msg.data)
        self.logger.debug(f"Received {motor_name} motor thrust: {msg.data}")

    def publish_status(self):
        # Имитируем реакцию робота на команды
        self.logger.info(f"Simulated status: Left = {self.left_thrust}, Right = {self.right_thrust}, Back = {self.back_thrust}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSimulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
