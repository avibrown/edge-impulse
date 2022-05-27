# Imports
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from edge_impulse_linux.runner import ImpulseRunner

# Classificiation node that subs to sensor stream and publishes to inference topic
class ClassificationNode(Node):
    def __init__(self):
        super().__init__("edge_impulse_classifier")

        # Sensor data subscriber
        self.subscriber_ = self.create_subscription(Float32MultiArray, "sensor_stream", self.callback_fill_buffer, 10) # 1: Change subscription

        # Inference publisher
        self.publisher_ = self.create_publisher(String, "inference_stream", 10)
        self.timer_ = self.create_timer(1, self.classify) # 2: Adjust timing

        # Create Classifier object based on Edge Impulse model file
        self.classifier = Classifier('modelfile.eim')
        
        self.buffer = []
        self.buffer_full_len = None #3: Set max length of buffer array

        self.get_logger().info("Edge Impulse node opened.")

    # If buffer is full, classify and publish result!
    def classify(self):
        # String since we're publishing the class name
        msg = String()

        if len(self.buffer) == self.buffer_full_len:
            # Pass buffer to classifier
            msg.data = self.classifier.classify(self.buffer)
            self.publisher_.publish(msg)

            # Reset buffer
            self.buffer = []

    # Adds to buffer whenever new sensor data is published
    def callback_fill_buffer(self, msg):
        if len(self.buffer) < self.buffer_full_len:
            for val in msg.data:
                self.buffer.append(val)


# Edge Impulse classification class
class Classifier:
    def __init__(self, model_path):
        self.runner = None
        self.model_path = model_path

    def classify(self, features):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        modelfile = os.path.join(dir_path, self.model_path)
        self.runner = ImpulseRunner(modelfile)

        try:
            self.runner.init()
            res = self.runner.classify(features)

            # Return classification with highest probability
            return max(res["result"]["classification"], key=res["result"]["classification"].get)

        finally:
            if (self.runner):
                self.runner.stop()

def main(args=None):
    rclpy.init(args=args)
    node = ClassificationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
