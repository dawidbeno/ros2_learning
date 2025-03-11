import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_action_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        # we create an action client using the custom action definition
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    # This method waits for the action server to be available, then sends a goal to the server. 
    # It returns a future that we can later wait on.
    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

# Creates an instance of our FibonacciActionClient node. 
# It then sends a goal and waits until that goal has been completed.
def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()