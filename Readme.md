# Running the System in a Dev Container

This branch implements a refactoring of the [Vortex ASV project](https://github.com/vortexntnu/vortex-asv), originally built for ROS 2 Humble, and updates it to support ROS 2 Jazzy.

---

## 1. Project Overview

This project uses [Dev Containers](https://code.visualstudio.com/docs/remote/containers) to create an isolated and fully configured development environment. All dependencies are managed within the container to ensure consistency across systems and developers.

---

## 2. Prerequisites

Before starting, make sure your system has:
- **Docker**
- **Visual Studio Code** (optional, but recommended)
- **Remote - Containers** extension for VS Code

---

## 3. How to Run the Container
> Note: The first time the build may take up to 30min to build
### 4.1 With VSCode


1. *Open the workspace rovOS*
2. *Enter CMD+SHIFT+P (Mac) or CTRL+SHIFT+P (Windows/Linux)*
3. *Search for* `>Dev Containers: Rebuild and Reopen in Container`

>Note: `Remote - Containers` must be installed

Alternatively you can
1. *Open the workspace rovOS*
2. *Open* `compose.yaml` *file*
3. *Click on* `Run all services` *button on top of the code*

### 3.2 Without VSCode

1. `cd` to the `rovOS` folder
2. run `docker compose up`

---

## 4. Accessing the GUI

Once the container is running, open your browser and navigate to:

[http://localhost:6080/](http://localhost:6080/)

This interface (based on noVNC) allows you to interact with the graphical environment inside the container.

---

## 5. Known Issues on Apple M1 (ARM)

On Apple M1 machines, you may encounter:

- **IntelliSense not resolving system headers** in C/C++
- False-positive errors in `.cpp` and `.h` files

### 5.1 Workaround 

Add the following to `src/ros2_wp/.vscode/c_cpp_properties.json`:
```json
"forcedInclude": [
    "${workspaceFolder}/.vscode/eigen_fix.h"
]
```

This ensures that IntelliSense preloads the necessary definitions for proper parsing.

---

## 6. Additional Resources

- [Official Dev Containers Guide](https://code.visualstudio.com/docs/remote/containers)
- [ROS2 Documentation](https://docs.ros.org/)
- [ros2_wp Setup README](https://github.com/Jacopix/rovOS/blob/main/src/ros2_wp/docs/README.md)

If you have any questions or run into issues, please contact the development team.

