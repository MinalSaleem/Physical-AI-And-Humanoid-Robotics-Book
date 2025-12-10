import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class PythonClient(Node):
    def __init__(self):
        super().__init__('python_client')
        self.cli = self.create_client(AddTwoInts, 'python_add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('python service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    python_client = PythonClient()
    if len(sys.argv) == 3:
        response = python_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        python_client.get_logger().info(
            'Result of python_add_two_ints: for %d + %d = %d' %
            (python_client.req.a, python_client.req.b, response.sum))
    else:
        python_client.get_logger().info('Usage: python3 minimal_client.py <a> <b>')
    
    python_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()