cmake_minimum_required(VERSION 3.8)
project(ros2_force_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForceSensorData.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# 查找SOEM库
# 设置SOEM的路径（相对于当前包的位置）
set(SOEM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

# 添加SOEM包含目录
include_directories(
  ${SOEM_ROOT}/include
  ${SOEM_ROOT}/build/include
  ${SOEM_ROOT}/osal
  ${SOEM_ROOT}/osal/linux
  ${SOEM_ROOT}/oshw/linux
)

# 查找SOEM静态库
find_library(SOEM_LIBRARY
  NAMES soem libsoem.a
  PATHS ${SOEM_ROOT}/build
  NO_DEFAULT_PATH
)

if(NOT SOEM_LIBRARY)
  message(FATAL_ERROR "SOEM library not found. Please build SOEM first.")
endif()

message(STATUS "Found SOEM library: ${SOEM_LIBRARY}")

# 创建可执行文件
add_executable(force_sensor_node src/force_sensor_node.cpp)

# 链接依赖
ament_target_dependencies(force_sensor_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 链接SOEM库
target_link_libraries(force_sensor_node 
  ${SOEM_LIBRARY}
  pthread
  rt
)

# 为了使用自定义消息，需要添加这个依赖
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(force_sensor_node "${cpp_typesupport_target}")

# 安装可执行文件
install(TARGETS
  force_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件和配置文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()