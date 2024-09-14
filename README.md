# Gazebo-Test-Sim
A ROS2-compatible simulation for perception and autonomy development.

## Setup/Usage
To run this project, you will need a ROS2 and Gazebo installation. __Currently, the easiest and best pairing includes ROS2 Jazzy and Gazebo Harmonic (both LTS versions), but requires an Ubuntu 24.04 install (or equivalent).__ It is also possible to pair ROS2 Humble with Gazebo Harmonic on Ubuntu 22.04 if necessary, although this pairing is not officially supported and may require manual compilation of other simulation packages. Some additional considerations include:
- Running the simulator in WSL is nearly unusable due to performance reasons. A native install or dual-boot is recommended.
- If using WSL, Gazebo releases prior to Garden do not have functional hardware-accelerated graphics (and software rendering is unusable for this project).
- The TrackedVehicle plugin used by this project does not work out-of-the-box with Gazebo releases prior to Harmonic.
- ROS2 Jazzy is capable of being built from source on Ubuntu 22.04, however, rosdep will not be functional, meaning dependencies will also have to built from source.

#### Full install and usage:

1. Follow the ROS2 install guide: [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html), [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
	- *To verify your installation, execute* `printenv | grep ROS`.
2. Choose a Gazebo version to pair with ROS, and install `ros_gz` integration libraries: https://gazebosim.org/docs/harmonic/ros_installation (this page lists all possible pairings and their level of support)
3. Install Gazebo: [Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu) (the release version can be changed using the dropdown menu)
	- *To verify your installation, try running Gazebo using* `gz sim`.
4. Create a new ROS workspace and clone this repo:
	```bash
	mkdir sim-ws && cd sim-ws
	mkdir src && cd src
	git clone https://github.com/S1ink/Gazebo-Test-Sim --recurse-submodules
	```
5. Make sure all required packages are installed using rosdep:
	```bash
	rosdep install --ignore-src --from-paths .
	```
6. Check the [Cardinal-Perception](temp_perception/README.md) readme to install all required dependencies (ie. GTSAM). Note that it is not required to build this project separately as it is built recursively with this one.
7. Build and source (execute when in workspace directory):
	```bash
	colcon build --symlink-install --executor parallel
	source install/setup.bash
	```
8. Run using a launch file (this is an example of the current "main" simulation):
	```
	ros2 launch lance_sim sim.launch.py
	```

#### Launching & Arguments

The project can be launched as a separate 'client' and 'server' (ex. on separate machines):
- __Server__ (gazebo, robot_state_publisher, ros_gz_bridge, lio/aruco, teleop, foxglove_bridge):
	```bash
	ros2 launch lance_sim sim_server.launch.py <use_gz_gui:={true/false}> <gz_map:={arena/maze/moon}>
	```
	"use_gz_gui" controls whether or not the gazebo client is launched, and "gz_map" controls which world file is loaded (although currently broken).

- __Client__ (joystick publisher, rviz):
	```bash
	ros2 launch lance_sim sim_remote.launch.py <rviz:={true/false}>
	```
	"rviz" controls wheather or not rviz is launched.

- __Combined__ (calls both client and server launchfiles):
	```bash
	ros2 launch lance_sim sim.launch.py <...>
	```
	All arguments for individual client/server launches are wrapped and can be used here as well.


## Resources (in no particular order)
- https://gazebosim.org/docs/harmonic/getstarted
- https://github.com/azeey/turtlebot3_simulations/tree/new_gazebo/turtlebot3_gazebo
- https://gazebosim.org/api/sim/8/tutorials.html
- https://github.com/ros-controls/gz_ros2_control
- https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1TrackController.html
- https://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html
- https://docs.foxglove.dev/docs/introduction

__*Last updated: 7/21/24*__