# Actions
Actions are a form of asynchronous communication in ROS 2. Action clients send goal requests to action servers. Action servers send goal feedback and results to action clients.

## Creating action interface
**1. create .action file**
Actions are defined in .action files of the form:
```
# Request
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```
- A request message is sent from an action client to an action server initiating a new goal.
- A result message is sent from an action server to an action client when a goal is done.
- Feedback messages are periodically sent from an action server to an action client with updates about a goal.

**2. update CMakeLists.txt**
Place these line in CMakeliests.txt, before `ament_package()` line:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

**3. update package.xml**
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

**4. Build and check it**
```
colcon build
source install/local_setup.bash
ros2 interface show custom_action_interfaces/action/Fibonacci

```

## Implementing action
Created new package `action_py` for implementign action server and client.

### Server
In the constructor for FibonacciActionServer, an action server is created with a callback that is called when a goal is accepted.
```
self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
```
By default, if the goal handle state is not set in the execute callback it assumes the aborted state.

**callbacks**
The execute_callback calculates the Fibonacci sequence up to order and publishes partial sequences as feedback as each item is added to the sequence.

### Client
In the constructor for FibonacciActionClient, an action client for the fibonacci action is created:
```
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```
Our action client will be able to communicate with action servers of the same action name and type.

**callback chain**
One of the most interesting aspects of this code is how it uses ROS2's asynchronous action client pattern with a chain of callbacks:
```
def send_goal(self, order):
    # ...
    self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    self._send_goal_future.add_done_callback(self.goal_response_callback)
```

This pattern demonstrates how ROS2 implements non-blocking communication. When the goal is sent, the client doesn't wait for completion but instead sets up a callback chain:
1. First, send_goal_async() returns a Future object
2. When the server responds to the goal request, goal_response_callback is triggered
3. This callback then sets up another Future with get_result_async()
4. When the action completes, get_result_callback is triggered
5. Meanwhile, feedback_callback can be called multiple times as the action progresses

This approach allows the node to remain responsive to other events while the action is in progress, which is particularly valuable for long-running tasks like computing large Fibonacci sequences.

**Future object**
A Future object in ROS2 is a programming construct that represents the result of an asynchronous operation that may not have completed yet. It's similar to Promise objects in JavaScript or Future/Promise constructs in other languages.

Key characteristics of Future objects:
1. Asynchronous Result Representation: A Future is a placeholder for a value that doesn't exist yet but will be computed and available at some later point in time.
2. Non-blocking Operations: Futures allow the code to continue execution without waiting for the operation to complete, making the system more responsive.
3. State Management: A Future can be in one of several states:
    - Pending: The operation hasn't completed yet
    - Completed: The operation has finished successfully
    - Failed: The operation has failed with an error
4. Callback Registration: As seen in the fibonacci_action_client.py, you can register callbacks using add_done_callback() that will be invoked when the Future completes.
