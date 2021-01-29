# Installation
### requirements:
system requirements:
```
	ros-melodic
	Ubuntu 18.04
	python3.6
```
ubuntu packages:
```
	sudo apt-get install ros-melodic-turtlebot3*
	sudo apt-get install ros-melodic-jackal*
```

python packages(in Anaconda Environment):
```
	conda install -c conda-forge libspatialindex=1.8.5
	pip install rtree plotly

```

### Install Cartographer
```
	sudo apt-get update
	sudo apt-get install -y python-wstool python-rosdep ninja-build stow
	
	mkdir catkin_ws
	cd catkin_ws
	wstool init src
	wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
	wstool update -t src

	sudo apt-get remove ros-melodic-abseil-cpp
	src/cartographer/scripts/install_abseil.sh
	catkin_make_isolated --install --use-ninja


```


# Execution

### Run Lidar SLAM
```
	roslaunch slamplanning mapping.launch
	cd src
	python path_planning.py
	python motion_planning.py
```

