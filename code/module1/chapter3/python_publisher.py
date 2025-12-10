import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonPublisher(Node):
    def __init__(self):
        super().__init__('python_publisher')
        self.publisher_ = self.create_publisher(String, 'python_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Python Agent Message: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Python Agent Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    python_publisher = PythonPublisher()
    rclpy.spin(python_publisher)
    python_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()