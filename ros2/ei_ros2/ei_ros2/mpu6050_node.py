import board
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import adafruit_mpu6050

class MPU6050Node(Node):
    def __init__(self):
        super().__init__("mpu6050")
        self.i2c = board.I2C()
        self.mpu = adafruit_mpu6050.MPU6050(self.i2c)
        self.publisher_ = self.create_publisher(Float32MultiArray, "mpu6050_stream", 15)
        self.frequency = 98
        self.timer_ = self.create_timer((1 / self.frequency), self.read_mpu6050)
        self.get_logger().info("MP6050 stream opened.")

    def read_mpu6050(self):
        msg = Float32MultiArray()
        msg.data = [round(self.mpu.acceleration[0], 2),
                    round(self.mpu.acceleration[1], 2),
                    round(self.mpu.acceleration[2], 2)]
        self.publisher_.publish(msg)

 
def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    rclpy.spin(node)
    rclpy.shutdown()