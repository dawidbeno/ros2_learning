import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    # The MinimalClientAsync class constructor initializes the node with the name minimal_client_async.
    # The constructor definition creates a client with the same type and name as the service node.
    # The type and name must match for the client and service to be able to communicate.
    # The while loop in the constructor checks if a service matching the type and name of the client is available once a second.
    # Finally it creates a new AddTwoInts request object.
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    # The send_request method will send the request and spin until it receives the response or fails.
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

# The main method constructs a MinimalClientAsync object
# sends the request using the passed-in command-line arguments
# calls rclpy.spin_until_future_complete to wait for the result
# logs the results
def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()