# Robotermodellierung (ROD)
Modeling a robotic system with a scara, a pillar and a six axis robot for a university project.
They should fulfill an industrial goal - putting packages into a box for a pharmaceutic use-case.
One of the robot, in this case the six-axis-robot, should be able to be positioned via an HMI.
They shall all work together in an automatic setup.
All of them are visualized in gazebo and rviz.

# Setup:
WSL ubuntu 20.04.
ROS noetic

## Specific Installations:
```bash
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-joint-trajectory-controller
sudo apt install ros-noetic-ros-controllers*
sudo apt install ros-noetic-moveit-ros-visualization
sudo apt install ros-noetic-position-controllers
sudo apt install ros-noetic-ros-control*
sudo apt install ros-noetic-moveit-commander
```

# Starting
Download GitHub repository:
```bash
git clone https://github.com/leoniebolt/rod.git
cd rod
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
Use the arrows to linearly move the sixaxis robot.

![image](https://github.com/user-attachments/assets/6df7e486-398f-464e-b476-8e3724ca9ff1)



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
