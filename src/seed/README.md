# SEED
Version 7.1 (ROS2 version + Docker)

This is a dockerized version of a ROS2 node implementing the SEED functionalities. 

## About SEED
SEED is an **attentional executive system** for robotics capable of online orchestrating and monitoring structured robotic tasks at different levels of abstraction. 
The system is basically composed of 3 modules:
1. Long-Term Memory (**LTM**): is a rocedural memory where hierarchical representation of all the available tasks are stored.
2. Working Memory (**WM**): is a volatile memory containing instantiated tasks from the LTM for possible execution.
3. Bheavior-Based System (**BBS**): is a repository of sensorimotor processes/procedures that can be recalled and executed by the robot.
During the execution, hierarchical tasks may be retrieved from the LTM and instantiated into the WM. The WM expands such tasks following the hierarchical specification loading additional subtasks into the memory. If retreived tasks are associated with a vehvior from the BBS (i.e., concrete tasks) the associated process is recalled.
Notice that SEED specifically allow multiple behaviors to be executed at the same time and exploits attentional regulations and contention scheduling for conflinct management (**competition**).

### Long-Term Memory (LTM)

### Working Memory (WM)

### Behavior-Based System (BBS)

### Competition



## Installation and execution via Docker (recomanded)

The whole SEED architecture is wrapped into a dockerized ROS2 node. Please check the [Docker]() and [ROS2]() guides for further details.

### Prerequisites (Docker installation)

Before to start, docker must be installed on your OS. For Ubuntu users you may refer to the following procedure:
```
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the Docker packages
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add docker user
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```


### Create Docker Image for SEED
In order to build the package you have to create an image for seed.
NOTE: the image may take up to 3Gb of memory on your drive.
```
./docker_build.sh [image_name]
# example:
./docker_build.sh seed_img
```


### Run a container
To run SEED a container for the created image must be started, you can used the provided script:
```
./docker_run.sh [image_name] [container_name]
# example:
./docker_run.sh seed_img seed_cnt
```
The shell will be automatically attached to the container. Notice that if you close this shell, the container will be closed as well.

If you want to attach a new shell to the previously started container you can use the following script:
```
./docker_attach.sh [container_name]
# example:
./docker_attach.sh seed_cnt
```


### Execution
To execute SEED simply type the following command into one of the attached shell:
```
ros2 run seed seed test
```
some default nodes will be loaded and expanded in the SEED working menmory.

To check the list of loaded nodes you can type the `listing` command into the shell.

To start the QT5 GUI you can type `gui` into the shell. 

To start the ROS2 semaphore test type `ros2 semaphore` into the shell (demo output is available in the `/semaphore/out` topic).



## Installation via apt-get (outdated and not recomanded)

This procedure is not recomanded as docker strongly simplifies installation and running on different platforms.

### Installation
In order to build the SEED package in the traditional way the following additional dependancies must be installed:
```
# standard SEED dependancies
sudo apt install swi-prolog
sudo apt install libgraphviz-dev
sudo apt install libqt5charts5-dev
sudo apt install libespeak-dev
```
Please refer to the **Dockerfile** for detailed installation instructions. 
Notice that SEED makes use of swi-prolog for the LTM functionalities, please refer to the offical [website](https://www.swi-prolog.org/) for further details.

### Execution
To execute SEED simply type the following command while a roscore is on:
```
ros2 run seed seed test
```
some default nodes will be loaded and expanded in the SEED working menmory.

To check the list of loaded nodes you can type the `listing` command into the shell.

To start the QT5 GUI you can type `gui` into the shell.




## Creating your SEED application

You may create a new SEED application by creating a new LTM or by customizing a preexisting one, but also by adding new behaviors to the BBS (add-ons). Remember that SEED is wrapped into a ROS2 node, you can either create your custom ROS2 interfaces (topics, services, etc.) within new behaviors or using bulit-in ones.    

### Create or modify a LTM
The SEED architecture is based on a hierarchical representation of tasks. Each task that can be either abstract (collection of further subtasks) or concrete (actual code). Parametric tasks are defined as prolog schemata collected into a specific LTM file from the LTM/ folder. LTM files must be named as follows:
```
seed_[YOUR_APP_NAME]_LTM.prolog
```
Where [YOUR_APP_NAME] is the name of your SEED application.

Inside the LTM, schemata are represented as follows:
```
seed_[YOUR_APP_NAME]_LTM.prolog
```

Different LTM files can be used to specify the set of tasks in the behavioral repository of the system.

### Add a new behavior to the BBS

### ROS2 interface (ros_behaviors)



# Acknowledgments



## Implementation
The SEED architecture is based on a hierarchical decomposition of tasks that can be either abstract (collectors for subtasks) or concrete (actual code). The hierarchical tasks are represented as prolog schemas collected into LTM files in the LTM/ folder. Different LTM files can be used to specify the set of tasks in the behavioral repository of the system. Default file is seed_LTM.prolog, in order to specify a different file you can run the SEED node as follows
```
ros2 run seed_pdt seed [NAME]
```
The variable [NAME] is used to identify this SEED instance, it automatically load a seed_[NAME]_LTM.prolog file from the LTM/ folder as the repository for that SEED instance.

The nodes that are loaded by default are specified into the "alive" schema.

In order to create a new abstract node you can just add a new schema into the LTM and compose it with other schemata if needed.

In order to create a new concrete node, a new cpp class must be created by implementing the virtual Behavior class. The file countaining the class must be included into the seed_wakeUp.cpp file and the class must be added into the specified list of behaviors (new if entry).

## SEED and Prsima-Drone-Team (PDT) instructions

In the PDT domain, there are two instances of SEED to be called: one for the rover and another for the drone. Both will have their dedicated GUI and task representation. In order to run the, open two terminals:
- in Terminal1 (drone-side) run the command:
```
ros2 run seed_pdt seed pdt_drone
```
- in Terminal2 (rover-side) run the command:
```
ros2 run seed_pdt seed pdt_rover
```
In both cases the GUI should open automatically and the agents should start with the assigned tasks. The output of SEED is published as a string message on the topic /seed_pdt_[AGENT]/command where [AGENT] can be drone or rover.

The input of the system is mainly provided by observing the TF. A tfobserver behavior implements a rule-based system to retrieve regulations and state variables from TF. Rules are specified in the .rules files inside the observer/ direcroty.
