# VKC-Demo

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

``` bash
source <path-to-src>/devel/setup.bash
roslaunch vkc_example <example-name>.launch
```

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

