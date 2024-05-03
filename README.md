# ROS2 Sample Project
Components for ROS2 Sample Project

## Documentation

Documentation is available under doc folder:
- *ros2*: documentation of components written for ROS2 (humble version)
- *usecase*: some sequence diagrams to depict the relation of components
- *doxygen*: components documented using doxygen (https://www.doxygen.nl/). 

The doxygen documentation can be compiled in /doc/doxygen folder, using
```bash
doxygen doc/Doxyfile 
```
It can be viewed using [/doc/doxygen/html/index.html](/doc/doxygen/html/index.html)

## TL;DR

### Dependencies

#### ROS 2

[ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  
[ROS 2 DDS tuning](https://docs.ros.org/en/foxy/How-To-Guides/DDS-tuning.html)

```bash
# in ubuntu, before you should enable multiverse repository

# Build deps
apt install python3-colcon-common-extensions python3-vcstool python3-rosdep ros-humble-irobot-create-msgs

# Cyclone DDS
apt install ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp
```
# Install

You should now run install script to build all components:
```bash
cd PoC
./install.sh
```

The script will list the components that need to be compiled to run,
enter the folders and follow compilation instructions.

# Run
To launch a simulation execute from the root of the repository:

## Simulation
You can use enable or disable automatic dock after reaching goal.    
`$DOCK` can be 1 (disable), 2 (enable).

```bash
./PoC/rumbo_run.sh dock_enable:=$DOCK
```

## AP Engine

To compile and execute the AP engine , you should: 
- follow instructions on [/PoC/AP_Engine/README.MD](/PoC/AP_Engine/README.MD)
- or run on a new terminal window (but with logs deleted after each round):
```bash
cd PoC/AP_Engine
./ap_run.sh
```

You can also pass as first argument (`true` or `false`) to clean build directory, as below:
```bash
cd PoC/AP_Engine
./ap_run.sh true
```

## Use case

### Goal
To create a new goal with *X=`$POS_X`, Y=`$POS_Y`, YAW=`$ORIENT`* (all coordinates are float values), you can run:
```bash
cd Storage
./create_goal.sh "$POS_X;$POS_Y;$ORIENT"
```

example:
```
./create_goal.sh "1.0;2.0;3.14"
```

### Abort
To abort a current with *ID=`$GOAL_CODE`*, you can run:
```bash
cd Storage
./create_abort.sh $GOAL_CODE
```

example:
```
./create_abort.sh GOAL-123456789
```
