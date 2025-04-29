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
