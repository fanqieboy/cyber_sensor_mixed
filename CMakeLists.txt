cmake_minimum_required(VERSION 3.5)
project(cyber_sensor_mixed)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------
# 1. 查找依赖包 (Find Packages)
# -----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(Threads REQUIRED) 

# -----------------------------------------------------------
# 2. 构建可执行文件 (Build Executable)
# -----------------------------------------------------------
set(SOURCE_FILES
  src/cyber_sensor_mixed_node.cpp
  src/udp_socket.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}_node 
  Threads::Threads  # C++ std::thread 支持
  rt                # 额外的系统库
  dl
)

# -----------------------------------------------------------
# 3. 安装规则 (Installation Rules)
# -----------------------------------------------------------

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch_ROS2
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch.py"
  PATTERN ".*" EXCLUDE
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.rviz"
  PATTERN ".*" EXCLUDE
)

# -----------------------------------------------------------
# 4. ROS 2 特定配置 (Ament Configuration)
# -----------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  std_msgs
  sensor_msgs
)

ament_package()
