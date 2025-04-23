# Development and Build Guide

This document contains the essential commands and steps for resolving dependencies, building the ROS2 packages, running tests, and sourcing the environment.

---

## RESOLVE DEPENDENCIES

### ROS2 Packages
Install ROS package dependencies using `rosdep`:

```bash
rosdep install --from-paths src -y --ignore-src
```

### Python Libraries
Install the required Python libraries with `pip`:
```bash
pip install -r requirements.txt
```

## BUILD PACKAGES
 Note: The --symlink-install option allows the installed files to be changed by modifying the files in the source space (e.g., Python files or other non-compiled resources), which enables faster iteration.
To build a specific package, run:

```bash
colcon build --symlink-install --packages-select <package_name>
```

## RUN TESTS
Execute the tests with:

```bash
colcon test
```

## SOURCE
After building, source the setup file to update your environment:

```bash
source install/setup.bash
```

## RUN
After sourcing, run the node you need

```bash
ros2 run <package_name> <node_executable_name>
```
---

Feel free to modify any sections as needed for your team's workflow!

Notes:

Ignore pcl and eigen3 when using rosdep. They are installed, but rosdep can't find them
rosdep install --from-path src -y --ignore-src --skip-keys pcl --skip-keys Eigen3

Add installation path for stonefish. Otherwise CMake will fail
Not sure which of the following works better
export Stonefish_DIR=/home/ubuntu/Desktop/ROS2/third-party/stonefish
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/home/ubuntu/Desktop/ROS2/third-party/stonefish/install

Install OpenGLM with the followint
sudo apt update
sudo apt install libglm-dev
