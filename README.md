# morus_uav_gazebo
Repository for developing UAV simulator based on Gazebo.

# Installation Instructions

## Configure workspace

This instructions consider you have [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) installed. Following dependencies have to be installed before configuring the workspace:

```sudo apt-get install python-wstool python-catkin-tools ros-kinetic-octomap-ros ros-kinetic-dynamic-edt-3d libssh2-1-dev unzip libyaml-cpp0.5v5 libblas-dev liblapack-dev```

Next, initialize workspace using catkin tools:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/kinetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd ~/catkin_ws
catkin build
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Install required simulation packages

If you don't have git you can install it with:
```sudo apt-get install git```

Next, clone and checkout following packages in `src` folder:

```
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/rotors_simulator.git 
cd rotors_simulator
git checkout 97b3da2d02ab498b0c9d7a15d0297e72fe6b6482
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/mav_comm.git
cd mav_comm
git checkout de1b6294fa30f2c5fb892831bc86bd7ec8c08d00
```


Before you build, install following dependencies:

```
sudo apt-get install libopencv-dev
sudo apt-get install ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox ros-kinetic-mavros ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-robot-controllers ros-kinetic-joint-state-controller ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control ros-kinetic-hector-gazebo-plugins
catkin build
```

Finally, clone morus_uav_gazebo repository using git lfs:

```
sudo apt-get install curl
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install
cd ~/catkin_ws/src
git clone https://github.com/larics/morus_uav_gazebo.git
catkin build
```
