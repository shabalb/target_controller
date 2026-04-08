cmake_minimum_required(VERSION 3.8)
project(target_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)

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

add_executable(move_target src/move_target.cpp)
add_executable(drive_robot src/drive_robot.cpp)
add_executable(image_viewer src/image_viewer.cpp)
ament_target_dependencies(move_target rclcpp )
ament_target_dependencies(drive_robot rclcpp geometry_msgs)
ament_target_dependencies(image_viewer rclcpp sensor_msgs cv_bridge message_filters geometry_msgs)

include_directories(${OpenCV_INCLUDE_DIRS})

# --- Gazebo (gz) зависимости ---
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

target_link_libraries(move_target
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

target_link_libraries(image_viewer ${OpenCV_LIBS})

install(TARGETS move_target 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS drive_robot 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS image_viewer
  DESTINATION lib/${PROJECT_NAME})


ament_package()
