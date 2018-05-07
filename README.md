# sequoia_ros
ROS packages for OU's IGVC Self-Drive competition entry.

After cloning into a ROS workspace, run the following the command from the root of the workspace to install the necessary ROS package dependencies to make everything work correctly:

`rosdep install --from-paths src --ignore-src -r`

## Gazebo simulation of Sequoia

### Starting the Simulator

`roslaunch gazebo_ros empty_world.launch`

To load a custom Gazebo world file, pass its file path to the `world_name` argument:

`roslaunch gazebo_ros empty_world.launch world_name:=<complete path to world file here\>`

Spawn Sequoia in Gazebo with `spawn_sequoia.launch` in the `sequoia_ros` package:

`roslaunch sequoia_gazebo spawn_sequoia.launch`

### Sensor Data Topics

* `/fix`: GPS position given in latitude and longitude in the form of a `sensor_msgs/NavSatFix` message

* `/camera_sim/image_raw/*`: Standard group of image parameters from `image_proc`

* `/laser/scan`: LIDAR scan data in the form of a `sensor_msgs/LaserScan` message

* `/vehicle/twist`: The current measurement of the vehicle's speed and yaw rate in a `geometry_msgs/TwistStamped` message. The `linear.x` field contains the speed in m/s and `angular.z` contains the yaw rate in rad/s

### Controlling the Simulated Vehicle
Gazebo subscribes to the following `sensor_msgs/Float64` topics to simulate control inputs to the vehicle:

* `/vehicle/throttle_cmd`: Throttle percentage (0 to 1)

* `/vehicle/brake_cmd`: Brake torque in Newton-meters (0 to 1000)

* `/vehicle/steering_cmd`: Desired steering wheel angle in radians (-9.5 to +9.5)

Instead of directly publishing messages to the above topics, the `sequoia_twist_controller` package provides a system for performing closed loop speed control and kinematic transformation from yaw rate to steering wheel angle.

To run the twist controller:

`roslaunch sequoia_twist_controller sequoia_twist_controller.launch`

To use the twist controller, just publish a `geometry_msgs/Twist` message on the `/vehicle/cmd_vel` topic, and it will generate the appropriate brake, throttle and steering wheel angle commands to send to the simulator.

