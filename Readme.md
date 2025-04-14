# Running the System in a Dev Container

This project uses [Dev Containers](https://code.visualstudio.com/docs/remote/containers) to create an isolated and fully configured development environment.

## How to Run the Container

Follow these simple steps to start the system:

1. **Open the project in VS Code**  
   Make sure you have the _Remote - Containers_ extension installed.

2. **Start the container**  
   Run the following command in VS Code:
   > **Dev Containers: Reopen in Container**
   
   This will start the container using the configurations defined in `.devcontainer/devcontainer.json`.

## Accessing the GUI

Once the container is running, you can access the graphical interface by opening your browser and navigating to:

[http://localhost:6080/](http://localhost:6080/)

This will allow you to interact with the application running inside the container.

## Known Issues on Apple M1 (ARM) Processors

If you're using an Apple M1 (ARM-based) machine, you might experience issues with IntelliSense for C/C++ files.  
In particular, system headers might not be correctly resolved, resulting in a large number of false-positive errors in otherwise valid C/C++ files.

### Workaround

To fix this issue, you can force the inclusion of a compatibility header that helps IntelliSense correctly parse the files.  
Modify your `src/ros2_wp/.vscode/c_cpp_properties.json` and add the following entry inside your configuration:

```json
"forcedInclude": [
    "${workspaceFolder}/.vscode/eigen_fix.h"
]
```

This ensures that the necessary definitions are preloaded by IntelliSense, improving compatibility on M1 systems.

## Additional Information

For more details on how to run the code, please refer to our detailed setup guide available in our setup_dev file. This file contains information on workspace settings, build options, and other developer-specific configurations:

[ros2_wp/docs/setup_dev.md](https://github.com/Jacopix/rovOS/blob/main/src/ros2_wp/docs/setup_dev.md)


---

If you encounter any issues or have questions, refer to the VS Code Dev Containers documentation or reach out to the development team.