add_executable(d_rover1 src/d_rover1.py)
add_executable(d_rover2 src/d_rover2.py)
add_executable(d_rover3 src/d_rover3.py)
add_executable(d_rover4 src/d_rover4.py)
add_executable(basestation src/basestation.py)
add_executable(launch all_nodes.launch.py)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverInfo.msg"
)

ament_export_dependencies(rosidl_default_runtime)

