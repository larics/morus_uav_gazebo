# morus_uav_gazebo
Repository for developing UAV simulator based on Gazebo.

# Running Simulation
It is expected that you have fulfilled all of the installation instructions on the master branch.
To switch to the side branch use:
```
git checkout morus_control_flap
```
Before you can start the simulation you have to cut the morus_plugins directory out of the morus_uav_gazebo directory and paste it in its parent directory which is the src directory. The second option is to try and alter the CMakeFiles.txt file in the morus_uav_gazebo/morus_description directory with add_subdirectory(morus_plugins).

Before we spawn the drone we must run the world launch command:
```
roslaunch gazebo_ros empty_world.launch 
```

You can run the simulation with detailed morus uav model by running following command:
```
roslaunch morus_gazebo spawn_morus_multirotor.launch
```
This will only spawn the drone. To start the three control files (height, attitude, position) use:
```
roslaunch morus_control morus_control.launch
```
