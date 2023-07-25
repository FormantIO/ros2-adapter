cmake_minimum_required(VERSION 3.8)
project(formant_ros2_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY scripts/converters DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/ros2_utils DESTINATION lib/${PROJECT_NAME})

execute_process(
  COMMAND bash -c "python3 -m pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/../requirements.txt"
)

# Install scripts
install(PROGRAMS
  scripts/main.py
  scripts/config.json
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
