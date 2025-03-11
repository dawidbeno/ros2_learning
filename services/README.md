# ROS2 service demo
Demo package to demonstrate how services in ROS2 work.
Created two nodes to request and respond to data over a service. Added their dependencies and executables to the package configuration files so that you could build and run them, allowing you to see a service/client system at work.

## Run the demo
1. Build the package
```
rosdep install -i --from-path py_srvcli --rosdistro jazzy -y
colcon build --packages-select py_srvcli
```
2. Open terminal, source setup and start service
```
source install/setup.bash
ros2 run py_srvcli service
```
3. Open new terminal, source setup and start client
```
source install/setup.bash
ros2 run py_srvcli client 2 3
```

### srv file
Services are defined in .srv files that specify both request and response message structures

Basic Structure:
- Two-part format separated by `---`
- Request part above the separator
- Response part below the separator
- Each part can be empty, but the **separator is required**

### server
The MinimalService node provides a basic ROS2 addition service that accepts two integers and returns their sum. It registers under the name 'minimal_service' and creates a service called 'add_two_ints' using the standard AddTwoInts interface. When a request arrives, it logs the input values and returns their sum as the response.

### client
The MinimalClientAsync node connects to the 'add_two_ints' service and sends asynchronous requests with two integers provided as command-line arguments. It waits for service availability before sending the request, then spins until receiving a response. Upon completion, it logs the result of the addition operation and terminates.

### package setup
Had to add these entiry point to setup.py
```
'service = src.service_member_function:main',
'client = src.client_member_function:main',
```
Moreover, the `src` directory has to contain `__init__.py` file, to be recognized as python package
