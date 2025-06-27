# ROD – Roboter-Station zur Verpackung und Palettierung

## Projektbeschreibung

Dieses Projekt simuliert eine Verpackungs- und Palettierstation in Gazebo unter ROS Noetic  
Ziel ist es, eine Verpackungsline mit mehreren Robotern realitätsnah darzustellen:

- Ein *SCARA-Roboter* verpackt Objekte von einem Förderband in eine Box.
    
- Ein weiterer Roboter setzt einen *Deckel* auf die befüllte Box.
    
- Ein *SixAxis-Roboter* (an ABB IRB 1100 angelehnt) stapelt die geschlossenen Boxen auf eine Palette.
    

Die Simulation basiert auf einem selbst erstellten URDF-Modell und wurde mit SolidWorks 2025 modelliert.

---

##  Systemvoraussetzungen

## Systemvoraussetzungen

- Betriebssystem: Ubuntu 20.04.6 LTS
    
- ROS: Noetic
    
- Python: 3.8.10
    
- Gazebo: 11.11.0
    
- RViz: 1.14.20 (aus ROS Noetic)
    
- MoveIt: 1.1.9
    
- CMake: 3.16.3
---

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

Doxygen dokumentation is in /docs/html/index.html
