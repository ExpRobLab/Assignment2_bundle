
## Installation (bundle workspace)
<details>
<summary>Details</summary>
This repository is provided as a bundle (multiple packages + repos files). The recommended workflow is to create a workspace and use vcs to import the referenced repositories.

Clone the assignment bundle (example)

```bash
sudo apt-get update
sudo apt install -y \
  python3-pip ros-dev-tools\
  ros-$ROS_DISTRO-control-msgs \
  ros-$ROS_DISTRO-control-toolbox \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  ros-$ROS_DISTRO-joy \
  ros-$ROS_DISTRO-teleop-twist-joy \
  ros-$ROS_DISTRO-moveit \
  ros-$ROS_DISTRO-moveit-ros-planning \
  ros-$ROS_DISTRO-moveit-ros-move-group \
  ros-$ROS_DISTRO-moveit-core \
  ros-$ROS_DISTRO-moveit-plugins \
  ros-$ROS_DISTRO-tf2 \
  ros-$ROS_DISTRO-tf2-ros \
  ros-$ROS_DISTRO-tf2-geometry-msgs \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-urdf\
  ros-$ROS_DISTRO-moveit-servo\
  ros-$RIS_DISTRO-moveit-msgs\
  ros-$ROS_DISTRO-localization\

sudo apt update
sudo apt install python3-vcstool
https://github.com/ExpRobLab/assignment2.git
git clone https://github.com/ExpRobLab/assignment2_bundle.git assignment2_ws
cd assignment2_ws
mkdir src
```

Import repositories (if the bundle supplies .repos files inside the cloned repo) from within your workspace:

```bash
vsc import src < assignment2_https.repos
```

or with ssh 

```bash
vsc import src < assignemnt2_ssh.repos
```

build

```bash
colcon build --symlink-install --packages-up-to assignment2 bme_gazebo_basics worlds_manager aruco_opencv_msgs aruco_opencv

source install/local_setup.bash
```


NOTE: The repo references ros_aruco_opencv external package. Make sure that package is available in your src (installed through `vcs`) and that you check-out (or initially clone in vs) a branch compatible with your ROS 2 distro if necessary. The package maintainer may have a branch per ROS distro. If using Humble or Jazzy, check out to the matching branch.

</details>
