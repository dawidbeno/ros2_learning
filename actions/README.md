# Actions
Actions are a form of asynchronous communication in ROS 2. Action clients send goal requests to action servers. Action servers send goal feedback and results to action clients.

## Creating action
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

