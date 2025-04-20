cmake_minimum_required(VERSION 3.5)
project(vortex_msgs)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/Clusters.msg"
  "msg/ThrusterForces.msg"
  "msg/Pwm.msg"
  "msg/KMBinary.msg"
  "msg/Landmark.msg"
  "msg/LandmarkArray.msg"
  "msg/OdometryArray.msg"
  "msg/Parameter.msg"
  "msg/ParameterArray.msg"
  "msg/HybridpathReference.msg"
  "msg/LOSGuidance.msg"
  "msg/ReferenceFilter.msg"
  "msg/DVLAltitude.msg"
  "msg/Waypoints.msg"
  "msg/Transducer.msg"
  "msg/TransducerArray.msg"
)

set(srv_files
  "srv/MissionParameters.srv"
  "srv/Waypoint.srv"
)

set(action_files
  "action/FilteredLandmarks.action"
  "action/FindDock.action"
  "action/GoToWaypoint.action"
  "action/LocateDock.action"
  "action/NavigateWaypoints.action"
  "action/ReferenceFilterWaypoint.action"
  "action/VtfGuidance.action"
  "action/LOSGuidance.action"
  "action/FilteredPose.action"
  "action/HybridpathGuidance.action"
  "action/WaypointManager.action"
)

# Generate ROS 2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    ${action_files}
    DEPENDENCIES std_msgs
      geometry_msgs
      nav_msgs
      shape_msgs
      sensor_msgs
 )

# Declare package dependencies
ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  std_msgs
  nav_msgs
  shape_msgs
  sensor_msgs
)

ament_package()

# Omg hello 😊
# Can we be friends, jk lul hihi 🤭
# Unless... 😳👉👈
