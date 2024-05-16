# ros_micro_bridge
ROS2 package that connects the microprocessor with ROS - A module of the on board vessel system

As the title says, this part of the repository is dedicated to the software bridge that translates commands from a control system (probably in MATLAB or Python) to the Arduino, and sensor data back from Arduino to the control system.

## Functionality
This basic functionality of this script is to pass on actuation data from ROS to the arduino and periodically receive and publish telemetry.
Further functionalities that increase reliability, maintainability and ease-of-use are:
* Validating integrity of serial messages
* Receiving and displaying Arduino diagnostics
* Show diagnostics of the script itself
* Manage various timeouts, such as stopping the actuators when a reference signal stopped broadcasting for some time. 
* Catch and/or solve various system failures (e.g. reboot the Arduino if is suspected to have crashed)
* Automated selection of Arduino serial port
* Filter raw magnetometer data to form heading estimate which is then published.

## Using this package
Start it directly:
```shell
# Make sure to have the ROS environment correctly set up
ros2 run ras_low_level_bridge arduino_bridge_ros2
```
Or incorporate this into a launch structure. 


A requirement is the VESSEL_ID variable set
```shell
export VESSEL_ID=RAS_TN_DB
```