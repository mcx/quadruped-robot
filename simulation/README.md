# Introduction

Here are the ROS simulation packages of Unitree robots, You can load robots and joint controllers in Gazebo, so you can do low-level control(control the torque, position and angular velocity) on the robot joints. Please watch out that the Gazebo simulation cannot do high-level control, namely walking. Besides of these simulation functions, you can also control your real robots in ROS by the [unitree_ros_to_real](https://github.com/unitreerobotics) packages. For real robots, you can do high-level and low-level control by our ROS packages.

## Packages:

Robot description: `go1_description`, `a1_description`, `aliengo_description`, `laikago_description`

Robot and joints controller: `unitree_controller`

Simulation related: `unitree_gazebo`, `unitree_legged_control`

# Dependencies

* [ROS](https://www.ros.org/) melodic or ROS kinetic(has not been tested)
* [Gazebo8](http://gazebosim.org/)
* [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real): `unitree_legged_msgs` is a package under [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real).

# Build

<!-- If you would like to fully compile the `unitree_ros`, please run the following command to install relative packages. -->

If your ROS is melodic(or higher version):

```
sudo apt-get install ros-${ros_version}-controller-interface  ros-${ros_version}-gazebo-ros-control ros-${ros_version}-joint-state-controller ros-${ros_version}-effort-controllers ros-${ros_version}-joint-trajectory-controller
```

Else if your ROS is kinetic:

```
sudo apt-get install ros-kinetic-controller-manager ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-joint-state-controller ros-kinetic-effort-controllers ros-kinetic-velocity-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-robot-state-publisher ros-kinetic-gazebo8-ros ros-kinetic-gazebo8-ros-control ros-kinetic-gazebo8-ros-pkgs ros-kinetic-gazebo8-ros-dev
```

And open the file `unitree_gazebo/worlds/stairs.world`. At the end of the file:

```
<include>
    <uri>model:///home/unitree/catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```

Please change the path of `building_editor_models/stairs` to the real path on your PC.

Then you can use catkin_make to build:

```
cd ~/catkin_ws
catkin_make
```

If you face a dependency problem, you can just run `catkin_make` again.

# Detail of Packages

## unitree_legged_control:

It contains the joints controllers for Gazebo simulation, which allows users to control joints with position, velocity and torque. Refer to "unitree_ros/unitree_controller/src/servo.cpp" for joint control examples in different modes.

## The description of robots:

Namely the description of Go1, A1, Aliengo and Laikago. Each package include mesh, urdf and xacro files of robot. Take Laikago as an example, you can check the model in Rviz by:

```
roslaunch laikago_description laikago_rviz.launch
```

## unitree_gazebo & unitree_controller:

You can launch the Gazebo simulation by the following command:

```
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs
```

Where the `rname` means robot name, which can be `laikago`, `aliengo`, `a1` or `go1`. The `wname` means world name, which can be `earth`, `space` or `stairs`. And the default value of `rname` is `laikago`, while the default value of `wname` is `earth`. In Gazebo, the robot should be lying on the ground with joints not activated.

### Stand controller

After launching the gazebo simulation, you can start to control the robot:

```
rosrun unitree_controller unitree_servo
```

And you can add external disturbances, like a push or a kick:

```
rosrun unitree_controller unitree_external_force
```

### Position and pose publisher

Here we showed how to control the position and pose of robot without a controller, which should be useful in SLAM or visual development.

Then run the position and pose publisher in another terminal:

```
rosrun unitree_controller unitree_move_kinetic
```

The robot will turn around the origin, which is the movement under the world coordinate. And inside of the source file `move_publisher`, we also offered the method to move robot under robot coordinate. You can change the value of `def_frame` to `coord::ROBOT` and run the catkin_make again, then the `unitree_move_publisher` will move robot under its own coordinate.
