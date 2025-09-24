# FSRDigitalTwin3D-ROS

ROS2 Humble Workspace for [FSRDigitalTwin](https://github.com/Neroware/FSRDigitalTwin3D)

## Setup
1. Setup ROS2 Humble on Ubuntu Jammy. We recommend using this [script](https://git.faps.uni-erlangen.de/robotik-public/una-unified-arbeits-umgebung/shell-script-una-ros2-and-vs-code-basic-install)!

2. Source ROS2 installation
```
source /opt/ros/humble/install.sh
```

3. Install Gazebo Classic or Ignition
```
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-ros-ign
```

4. Clone repo and set up Colcon Workspace
```
export COLCON_WS=<path/to/this/repo>
```

5. Install Robotnik Controller
```
cd $COLCON_WS/src/
sudo dpkg -i ./robotnik_simulation/debs/ros-humble-robotnik-controllers*.deb
```

6. Install ROS dependencies
```
cd $COLCON_WS
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

7. Compile and get a coffee
```
colcon build
source install/setup.sh
```

## Launch

### UR5e Moveit
If you want to enable MoveIt trajectory planning over TCP/IP, open 2 terminals and use the following two commands:
```
ros2 launch ur5e_moveit robot.ur5e_sim.launch.xml
ros2 launch ur5e_moveit test.pnp_mover.launch.py tcp_port:=9090 tcp_ip:=0.0.0.0
```

### Robotnik Simulation
If you want to run the Gazebo simulation from Robotnik, we refer to their [repo](https://github.com/RobotnikAutomation/robotnik_simulation/tree/humble-devel).
