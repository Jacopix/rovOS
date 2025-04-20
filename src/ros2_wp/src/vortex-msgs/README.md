# vortex_msgs


[![Industrial CI](https://github.com/vortexntnu/vortex-msgs/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/vortex-msgs/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/vortex-msgs/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/vortex-msgs/main)

This ROS package contains all custom ROS 2 interfaces used in all workspaces

### Before you add a new custom interface: are you _really_ sure you need one?
First have a look in the [common_interfaces](https://github.com/ros2/common_interfaces) repository to find out if any of the message and service types included in ROS 2 works for you.

### Adding a new custom interface
New message definitions are added to [msg](msg).

Add the new message in the [CMakeLists.txt](CMakeLists.txt) file.
```cmake
set(msg_files
   ... # other messages
   "msg/YourNewMessage.msg"
)
```

For services and actions it is the same way. Service definitions are added to [srv](srv), and to the [CMakeLists.txt](CMakeLists.txt) like
```cmake
set(srv_files
   ... # other services
   "srv/YourNewService.srv"
)
```

Action definitions are added to [action](action), and to the [CMakeLists.txt](CMakeLists.txt) like
```cmake
set(action_files
   ... # other actions
   "action/YourNewAction.action"
)
```

### Using the messages in your package
In your `package.xml`, add this dependency
```xml
<depend>vortex_msgs</depend>
```
Add it to your `CMakeLists.txt`
```cmake
find_package(vortex_msgs REQUIRED)
```
If your package includes C++ code which depend on vortex_msgs you also need to add the following to the `CMakeLists.txt`
```cmake
ament_target_dependencies(<your_executable>
    ... # other dependencies
    vortex_msgs
)
```

### Include the interfaces in your code
For C++
```cpp
// message
#include <vortex_msgs/msg/your_new_message.hpp>

// service
#include <vortex_msgs/srv/your_new_service.hpp>

// action
#include <vortex_msgs/action/your_new_action.hpp>
```

For Python
```python
# message
from vortex_msgs.msg import YourNewMessage

# service
from vortex_msgs.srv import YourNewService

# action
from vortex_msgs.action import YourNewAction
```
