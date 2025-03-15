from custom_action_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class FibonacciActionClient(Node):
    # Initializes the ROS2 node named 'fibonacci_action_client' and creates an ActionClient
    # for the Fibonacci action type.
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    # Creates and sends a goal message with the specified Fibonacci order to the server,
    # sets up callback chains for responses and feedback.
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Handles the server's response to a goal request,
    # logging whether the goal was accepted and setting up the result callback if successful.
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Processes the final result from the completed action,
    # displaying the Fibonacci sequence and shutting down the node.
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    # Processes intermediate feedback messages from the server,
    # displaying partial Fibonacci sequences as they're calculated.
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


# Initializes ROS2, creates the client, sends a goal requesting a Fibonacci sequence of order 10,
# and starts the event loop.
def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
