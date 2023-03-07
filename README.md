# Sensing Aruco ROS Packages
A set of ROS packages for detecting ArUco markers.

## Prerequisites

1) Uninstall opencv-python if it is installed. Having opencv-python and opencv-contrib-python will conflict with each other.
```
# Use the following for Python 3.
python3 -m pip uninstall opencv-python
```

2) Install ROS geometry msgs, std msgs, and cv-bridge packages.
```
# Use the following for ROS Noetic.
sudo apt-get install ros-noetic-geometry-msgs ros-noetic-std-msgs ros-noetic-cv-bridge
```

3) Install the following Python packages.
```
# Use the following for Python 3.
python3 -m pip install opencv-contrib-python>=4.6.0.66 scipy>=1.9.0 --user
```

4) Install [sensing-aruco](https://github.com/troiwill/sensing-aruco) Python package using the linked instructions ([link](https://github.com/troiwill/sensing-aruco/blob/main/README.md#installation)).

## Installation

Perform the following steps to install the ROS packages.
1) Clone the sensing-aruco repository.
```
cd <catkin workspace>/src
git clone https://github.com/troiwill/sensing-aruco-ros
cd sensing-aruco-ros
```

2) Make the scripts executable.
```
chmod +x sense_aruco_ros/nodes/aruco_detector_node
```

3) Build the workspaces.
```
cd ..
catkin build
```

4) Source the catkin workspace.
```
source devel/setup.bash
```

## Run Gazebo Demo

Run the Gazebo demo.
```
# In one terminal tab, run:
roslaunch sense_aruco_simulations run_marker_sim_demo.launch

# In a second terminal tab, run the command to ensure the detector node works:
rostopic echo /aruco_marker_poses
```
