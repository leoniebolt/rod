# Robotermodellierung (ROD)
Modeling a robotic system with a scara and a six axis robot for a university project.
They should fulfil an industrial goal - putting packages into a box for example for a pharmaceutic use-case.
One of the robot, in this case the six-axis-robot, should be able to be positioned via an HMI.
Both of them are visualized in gazebo and rviz.

# Installation
Basic Setup:
install ubuntu 20.04.
install ros noetic

Specific Installations:
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-joint-trajectory-controller
sudo apt install ros-noetic-ros-controllers*
sudo apt install ros-noetic-moveit-ros-visualization
sudo apt install ros-noetic-position-controllers
sudo apt install ros-noetic-ros-control*
sudo apt install ros-noetic-moveit-commander

# Starting
Download GitHub repository:

```bash
git clone
cd
```

In workspace folder:
```bash
catkin_make
source devel/setup.bash
```

For launching the HMI:
```bash
roslaunch rod_moveit hmi.launch
```

Use the arrows to linearly move the sixaxis robot


For launching the automatic program:
```bash
roslaunch rod_moveit launchfile.launch
```


In case of having troubles due to a dying node, put this in your terminal:
```bash
chmod +x ~/rod_moveit/scripts/hmi.py
chmod +x ~/rod_moveit/scripts/control_listener_sixaxis.py
```

This should resolve the issue.
