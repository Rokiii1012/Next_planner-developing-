cmake_minimum_required(VERSION 3.8)
project(wind_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)


# 生成可执行文件
add_executable(run src/run.cpp)

# 生成库
add_library(wind_planner src/wind_planner.cpp)
add_library(planner src/planner.cpp)

# 编译所需的头文件的位置
include_directories(include
/usr/local/include/OsqpEigen
/usr/local/include/osqp
/opt/ros/humble/include
${EIGEN3_INCLUDE_DIRS}
)

# 编译库，库对应的头文件也需要这行命令找到头文件
# target_include_directories(run PUBLIC
#   include
#   )
# target_include_directories(wind_planner PUBLIC
#   include
#   )

target_compile_features(run PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# 与第三方库进行链接
ament_target_dependencies(
  wind_planner
  "rclcpp"
  "nav_msgs"
  "Eigen3"
  "geometry_msgs"
)
ament_target_dependencies(
  planner
  "rclcpp"
  "nav_msgs"
  "Eigen3"
  "geometry_msgs"
)
# 与自定义库进行链接
target_link_libraries(run wind_planner planner OsqpEigen::OsqpEigen osqp::osqp) 
# 将可执行文件和库和头文件安装到指定目录，bin可执行文件，lib静态库，share动态库，include头文件
# install(TARGETS run
# RUNTIME DESTINATION lib${PROJECT_NAME})

# install(TARGETS wind_planner
# LIBRARY DESTINATION lib${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include/
# )
# 只有将可执行文件这样设置，ros2 run才能够找到可执行文件
install(TARGETS run
DESTINATION lib/${PROJECT_NAME})

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

# 项目设置由 ament_package() 完成，每个软件包必须恰好调用一次。
# ament_package() 安装了 package.xml，在ament索引中注册了软件包，并安装了CMake的配置（和可能的目标）文件，以便其他使用 find_package 的软件包可以找到它。
# 由于 ament_package() 从 CMakeLists.txt 收集了大量信息，因此它应该是您的 CMakeLists.txt 中的最后一个调用。
# ament_index
ament_package()
