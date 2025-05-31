cmake_minimum_required(VERSION 3.8)
project(dual_camera_msgs)

# Set standards to C++ 17
if(NOT CMAKE_CXX_STANDARD)
 	set(CMAKE_CXX_STANDARD 17)
endif()

# Define compiling options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define interfaces to generate
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ImagePair.msg"
	"msg/JsonString.msg"
    "msg/MonoIR.msg"
    "msg/StereoIR.msg"
    "msg/StereoVO.msg"
    DEPENDENCIES sensor_msgs
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
