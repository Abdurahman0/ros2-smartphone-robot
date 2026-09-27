cmake_minimum_required(VERSION 3.8)
project(smartphone_robot_interfaces)

# This package contains ONLY interface definitions (.action / .msg / .srv).
#
# It has to be an ament_cmake package even though the rest of the project is Python:
# .action files are compiled by rosidl into C, C++ and Python bindings, and that code
# generation only runs under CMake. Keeping interfaces in their own package is also
# standard practice -- it lets other packages depend on the message types without
# pulling in any node code, and avoids a circular dependency.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DriveDistance.action"
)

ament_package()
