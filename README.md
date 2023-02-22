# Planning-on-VKC

![ros_vesrion](https://img.shields.io/badge/ROS-Noetic-blue) ![sys-vesrion](https://img.shields.io/badge/Ubuntu-20.04-blue) 

A motion planning framework for virtual kinematic chain.

*Only tested with ROS Noetic for Ubuntu 20.04.*

## 1. Installation


### 1.1 Preparation
Firstly, following dependencies need to be manually installed and set up.
- [ROS Noetic](http://wiki.ros.org/noetic/Installation): we use ROS noetic as our basic platform.
<!-- - [Gurobi Optimizer](https://www.gurobi.com/downloads/gurobi-optimizer-eula/): For Gurobi download and licensed, you need to register an account first (Free academic use if you have an .edu email). An detail installation documentation is available [here](https://www.gurobi.com/documentation/). -->

After installing the aforementioned dependencies, follow steps below to setup the environment

Before compiling our package, several system dependencies needs to be installed, use following command to install

```bash
sudo apt install python3-catkin-tools ros-noetic-octomap-msgs ros-noetic-octomap \
ros-noetic-ompl ros-noetic-octomap-ros ros-noetic-lms1xx ros-noetic-ifopt
```

### 1.2 Clone repositories

We recommend to clone the repository instead of downloading files to avoid submodule errors.

Be sure you have [added ssh key to github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) to clone the submodules.

Cloning the repositories(especially submodules) may take up to hours, depend on your network connection.

```bash
cd projects
git clone -b release git@github.com:zyjiao4728/Planning-on-VKC.git --recurse-submodules
cd Planning-on-VKC
git submodule update --init --recursive --progress
rosdep install --from-paths src --ignore-src -r -y
catkin build
```

where `<github-package-url>` is the GitHub download URL of our package.

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
roslaunch vkc_example household_env.launch
```
![image](./src/pictures/vkc_pick_stick.gif)   ![image](./src/pictures/vkc_move_ball_with_stick.gif)    ![image](./src/pictures/vkc_open_cabinet_door.gif)

![image](./src/pictures/vkc_big_task_move_cup_with_plate.gif)    ![image](./src/pictures/vkc_big_task_move_all_cups_with_plate_into_cabinet.gif)    ![image](./src/pictures/vkc_big_task_move_chair_away_from_path_way.gif)
