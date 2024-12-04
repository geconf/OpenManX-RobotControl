# OpenManX-RobotControl
Final Project for WPI's RBE500-Foundations of Robotics. We built ROS2 Humble packages to control the forward, inverse, and velocity kinematics of the Open Manipulator X. A PD position controller was implemented to handle joint values.

## Build
Copy the contents of the entire repository into the src/ directory of a new ROS2 workspace.
```
mkdir -p rbe500_ws/src
cd rbe500_ws
git clone https://github.com/hylander2126/OpenManipulatorX_ROS2.git ./src
```

Run the following to build the packages in your new workspace:
```
colcon build --symlink-install
```
You may get some warning mesages, ignore them for now as long as everything successfully builds.


Don't forget to source your workspace:
```
source install/setup.bash
```

After connecting to the robot over USB, run the following to begin position control:
```
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

Now run the example code to move the robot:
```
ros2 run rbe500-example basic_robot_control
```
Or for python:
```
ros2 run rbe500_example_py basic_robot_control
```


---
RBE 500 - Foundations of Robotics 2023 taught by Professor Berk Calli at Worcester Polytechnic Institute Robotics Engineering Department
