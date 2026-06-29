cmake_minimum_required(VERSION 3.8)
project(outdoor_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------
# 本包为纯配置 / launch 功能包，无 C++ 源码需要编译
# 只需 ament_cmake 即可完成构建（install 步骤）
# 运行时依赖 (gazebo_ros / gazebo_plugins / sensor_msgs / rclcpp)
# 在 package.xml 中以 <exec_depend> 声明
# ---------------------------------------------------------------
find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------
# ★ 关键：install(DIRECTORY ...) 将三个资源目录安装到
#   install/outdoor_sim/share/outdoor_sim/ 下
# 这样 launch 文件、xacro 模型、world 文件才能被 ros2 launch 找到
# ---------------------------------------------------------------
install(DIRECTORY
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
