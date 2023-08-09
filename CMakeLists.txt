cmake_minimum_required(VERSION 3.8)
project(formant_ros2_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install python dependencies
execute_process(
  COMMAND bash -c "python3 -m pip install -r ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt"
)

# Find cmake dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install python modules
include(GNUInstallDirs)
install(DIRECTORY
  ${PROJECT_NAME}/scripts/
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  PATTERN "config_examples" EXCLUDE
)

# Keep existing install pattern to maintain backwards compatibility
# Can be removed in the future
install(DIRECTORY ${PROJECT_NAME}/scripts/ DESTINATION lib/${PROJECT_NAME} PATTERN "config_examples" EXCLUDE)

# Install launch files
install(DIRECTORY ${PROJECT_NAME}/launch
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/launch
  FILES_MATCHING
  PATTERN "*.launch.py"
  PATTERN "*.launch"
)

# Install scripts
# Keep existing config location to maintain backwards compatibility
# Can be removed in the future
install(PROGRAMS
  ${PROJECT_NAME}/scripts/config.json
  ${PROJECT_NAME}/scripts/main.py
  ${PROJECT_NAME}/scripts/ros2_adapter.py
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)

install(FILES
  ${PROJECT_NAME}/scripts/config.json
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
