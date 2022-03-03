# Planning-on-VKC

![ros_vesrion](https://img.shields.io/badge/ROS-Melodic-blue) ![sys-vesrion](https://img.shields.io/badge/Ubuntu-18.04-blue) 

A motion planning framework for virtual kinematic chain.

*Only tested with ROS Melodic for Ubuntu 18.04.*

## 1. Installation

Firstly, following dependencies need to be manually installed and set up.
- [ROS Melodic](http://wiki.ros.org/melodic/Installation): we use ROS melodic as our basic platform.
- [Gurobi Optimizer](https://www.gurobi.com/downloads/gurobi-optimizer-eula/): For Gurobi download and licensed, you need to register an account first (Free academic use if you have an .edu email). An detail installation documentation is available [here](https://www.gurobi.com/documentation/).

After installing the aforementioned dependencies, follow steps below to setup the environment

Create a ROS workspace: 

```bash
mkdir -p <ros-workspace-name>
```

where `<ros-workspace-name>` is the name of your newly created ROS workspace.



Before compiling our package, several system dependencies needs to be installed, use following command to install

```bash
sudo apt install python-catkin-tools ros-melodic-octomap-msgs ros-melodic-octomap \
ros-melodic-ompl ros-melodic-octomap-ros ros-melodic-lms1xx ros-melodic-ifopt
```



Then download our package

```bash
cd <ros-workspace-name>
git clone --recursive <github-package-url>
cd ../
catkin build --force-cmake -DTESSERACT_ENABLE_TESTING_ALL=OFF -DTESSERACT_ENABLE_TESTING_ALL=OFF
```

where `<github-package-url>` the GitHub download URL of our package.

*Note: Use `--dry-run` to list the packages which will be built.*



## 2. Run Examples

### 2.1 Examples come with tesseract package:

``` bash
source <path-to-src>/devel/setup.bash
roslaunch tesseract_ros_example <example-name>.launch
```

### 2.2 Examples come with vkc package:

#### 2.2.1 Example of using stick to pick ball:
The demo shows how a robot tries to pick an object with a tool and to operate articulated object such as opening the door of a cabinet.
``` bash
source <path-to-src>/devel/setup.bash
roslaunch vkc_example arena_env.launch
```
![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_pick_stick.gif)   ![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_move_ball_with_stick.gif)    ![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_open_cabinet_door.gif)
#### 2.2.2 Example of loading scene URDF with build-in module:
The demo shows vkc being able to modify scene graph after loading it from urdf to inverse the kinematics chain, and then plan a motion task basing on the new scene graph.
``` bash
source <path-to-src>/devel/setup.bash
roslaunch vkc_example load_vkc.launch
```
![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_urdf_pick_bottle_with_gripper.gif)    ![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_urdf_move_bottle_with_gripper.gif)
#### 2.2.2 Example of big task planning for complex tasks:
The demo provides as many as five scenarios of motion planning tasks, and they show different abilities of motion planning to finish one sub-task. Using tool-like objects, containers, as well as clearing obstacles from the moving path are the main idea of the demo that wants to convey.  Without modifying the source code and recompiling the project, it is still possible to switch among demo scenarios by changing the value of arg demo_index in the launch file vkc_big_task.launch. Comments are there to help you make decision. 
``` bash
source <path-to-src>/devel/setup.bash
roslaunch vkc_example vkc_big_task.launch
```
![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_big_task_move_cup_with_plate.gif)    ![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_big_task_move_all_cups_with_plate_into_cabinet.gif)    ![image](https://github.com/zyjiao4728/Planning-on-VKC/blob/master/src/pictures/vkc_big_task_move_chair_away_from_path_way.gif)
## 3. Trouble Shooting

### 3.1 Trajopt build test error:

Set default optimization solver to `BPMPD` :

``` bash
export TRAJOPT_CONVEX_SOLVER=BPMPD
```

If you have installed Gurobi, use `GUROBI` as your default optimizer:

``` bash
export TRAJOPT_CONVEX_SOLVER=GUROBI
```

