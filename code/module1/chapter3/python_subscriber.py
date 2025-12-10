import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonSubscriber(Node):
    def __init__(self):
        super().__init__('python_subscriber')
        self.subscription = self.create_subscription(
            String,
            'python_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Python Agent Heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    python_subscriber = PythonSubscriber()
    rclpy.spin(python_subscriber)
    python_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()