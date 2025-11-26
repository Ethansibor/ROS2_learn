cmake_minimum_required(VERSION 3.8)
project(robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

# 1. 模拟器节点
add_executable(simulator_node src/simulator_node.cpp)
ament_target_dependencies(simulator_node rclcpp interfaces)

# 2. 控制台节点
add_executable(console_node src/console_node.cpp)
ament_target_dependencies(console_node rclcpp interfaces)

# 3. 安装可执行文件
install(TARGETS
  simulator_node
  console_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 安装 Launch 文件 (C++包需要手动安装 launch 文件夹)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
