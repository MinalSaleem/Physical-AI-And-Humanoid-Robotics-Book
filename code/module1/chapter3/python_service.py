from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class PythonService(Node):
    def __init__(self):
        super().__init__('python_service')
        self.srv = self.create_service(AddTwoInts, 'python_add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Python Service server started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Python Service Incoming request: a: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    python_service = PythonService()
    rclpy.spin(python_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()