# Introduction
This is a dockerized version of the SEED_PDT package, implementing task execution functionalities for a team of robots involved in exploration and inspection activities.

This code is based on SEED version 6.0 (ROS2 version), please refer to the inner README (inside the `seed_pdt` dir) for further details about SEED. 

# Instructions
Here instructions to build and use the package.

### Installation and Build
This package works in ROS2 and makes use of dockerfile for the installation.

In order to build the package run the following command form the local folder:
```
# standard SEED dependancies
./docker_build.sh
```

NOTE: if changes are made to the files inside the src directory, the package has to be built again. 

### Execution (SEED-only)

#### Exploration inside Leonardo Arena
To run SEED considering the Leonardo-arena setting, type the following commands:
```
./docker_run.sh
ros2 launch seed_pdt seed_lnd.py
```

#### Exploration inside UNINA's Sala Alta Tensione (Testing)
To run SEED considering the UNINA setting (Sala alta tensione), type the following commands:
```
./docker_run.sh
ros2 launch seed_pdt seed_unina.py
```

#### Inspection inside UNINA's Sala Alta Tensione (Testing)
To run SEED considering the UNINA setting (Sala alta tensione), type the following commands:
```
./docker_run.sh
ros2 launch seed_pdt seed_inspect.py
```

### Execution (ros-bridge)
To use this package with ROS1 nodes the ros-bridge node must be used. The ros-bridge package should be installed from standard repository.

To facilitate ROS1/ROS2 commands, it is suggested to add the following macros at the end of your bashrc:
```
# Help functions to switch between ROS1 and ROS2
function ROS2 {
	export PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages
	source /opt/ros/foxy/setup.bash 
}
function ROS1 { 
	export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages
	source /opt/ros/noetic/setup.bash 
	source /home/[USERNAME]/catkin_ws/devel/setup.bash 
}
```
Where [USERNAME] is the name of the local user. Here we are assuming ROS2 foxy and ROS1 noetic installed. 

Assuming the previous macros have been added, the bridge can run from the current folder as follows:
```
ROS1
rosparam load bridge/bridge.yaml
ROS2
ros2 run ros1_bridge parameter_bridge
```


### Working Principles and Configuration Instructions
There are basically 3 configuration files:
1. A .py launch file in the src/seed\_pdt/launch/ directory (one for the whole team).
2. A .prolog file in the src/seed\_pdt/LTM direactory (one for each robot).
3. A .rules file in the src/seed\_pdt/observer directory (one for each robot).

NOTE: each time one of these files are changed, the Docker must be rebuilt.

#### Launch File
It is suggested to create a specific launch file for each team/scenario (e.g., `seed_lnd.py` specifies a team of 3 agents for the leonardo drone contest scenario on the real Leonardo arena). Inside this file, there will be $n$ SEED nodes one for each agent. For each on it is possible to specify the [NAME] (as argument) and a list of tf frames to be explored (as parameters), for example:
```
Node(
    package='seed_pdt',
    executable='seed',
    name='mimic',
    output="screen",
    arguments=["pdt_drone"],
    parameters=[
       {"frames_to_explore": ["expcen","expcl","expr2","expar","expal","exptwin","expsqr"]} #Leonardo-safe
    ]
)
```
launches one SEED instance for a drone named `pdt_drone` that is allowed to explore 7 frames, whose names are specified in the "frames\_to\_explore" list.

#### LTM File
Each robot must be associated to a .prolog file from the src/seed\_pdt/LTM direactory. The file must be named `seed_[NAME]_LTM.prolog` and contains the list of all tasks that the robot is able to execute. 

#### Rule File
Each robot must be associated to a .rule file from the src/seed\_pdt/observer directory. The file must be named `seed_[NAME].rules` and contains a list of variables that the robot has to observe during the execution. Rules may generate Boolean or Double variables.

1. Boolean varables: that can be used as state variables to enable/disable/finalize tasks. These are in the form:
```
[VALUE] [OP] [FUN]([TF1], [TF2]) -> [VAR_NAME]
```
where [VALUE] is a number, [OP] is an operator (">" or "<"), [FUN] is the name of a function to be applied to a pair of frames [TF1] and [TF2]. The complete list of functions and operator is available at `/ros2_seed/src/seed_pdt/include/behaviour/pdt_behaviours.h`, as methods of the TfObserverBehaviour class.

For example, the rule:
```
1.0 < zdiff(map,base_link) -> flying
```
specifies that the "flying" variable is considered true if the z-distance between the map frame and the base\_link frame is greater than 1.0 meters.

2. Double variables: that can be used to specify the level of priority of tasks. These are in the form:
```
[VALUE] [OP] [FUN]([TF1], [TF2]) ~> [VAR_NAME]
```
where [VALUE] is a number, [OP] is an operator ("/" or "*"), [FUN] is the name of a function to be applied to a pair of frames [TF1] and [TF2]. The complete list of functions and operator is available at `/ros2_seed/src/seed_pdt/include/behaviour/pdt_behaviours.h`, as methods of the TfObserverBehaviour class.

For example, the rule:
```
1 / xydist(base_link,exp00) ~> exp00.distance
```
specifies that the value of the variable "exp00.distance" increases as the x-y distance between the base\_link frame and the exp00 frame decreases. Therefore, a task having exp00.distance as regulatory factor will be priortized as the agent get close to the frame.

