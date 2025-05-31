# Development nodes
> Written for Nautilus subgroup *Sensing Rigs*

---

## Description

These nodes compose Sensig Rigs sub project, creating an operating system written in ROS 2 Jazzy.

Each package in this repository represent a specific task:
- **dual\_camera\_msgs**: creates a series of custom messages used by the system to communicate the results.
- **cv\_algorithms**: creates 3 LifeCycle nodes, one for each computer vision algorithm implemented, takes in input 2 synchronized frame (left and right) and each one return a custom output message. The implemented algorithms are:
    - **Monovision image recognition**, trained model to recognize crabs, requires only 1 frame in input.
    - **Stereovision image recognition**, trained model to recognize scallops.
    - **Stereovision visual odometry**, an algorithm that tries to create the trajectory of the module by detecting movements in the analysed frames.
- **normalizator**, takes in input every algorithms output and converts them in a .json format for the modem.

---

## Building and Running

To build the packages:
```shell
colcon build
```

To run the *normalizator* node:
```shell
ros2 launch normalizator normalizator_node_launch.py
```

To run the *cv\_algorithms* nodes:
```shell
ros2 launch cv_algorithms cv_algorithms_launch.py
```

To set the parameters of these nodes, modify the relative .yaml file under config/

---

## Known bugs

- **lc_stereo_vo_node** contains invalid calibration values, as a result it can't analyse properly the frames.
- the system doesn't handle keyboard interrupt.

---

## TODO

- [ ] Add launch file to initialize every node
- [ ] Integrate image acquisition nodes (the merge)
