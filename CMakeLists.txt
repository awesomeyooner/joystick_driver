cmake_minimum_required(VERSION 3.8)
project(joystick_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The Main Exec Name
set(MAIN_EXEC joystick_teleop)
set(MAPPINGS_EXEC create_mappings)

add_subdirectory(lib/plib)

# Create list of dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  std_msgs
  ament_cmake
  sensor_msgs
  backward_ros
  geometry_msgs
  yaml-cpp
  generate_parameter_library
)

# Find each dependency
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate Parameter Library
generate_parameter_library( ${PROJECT_NAME}_parameters
  src/parameters.yaml
)

set(SRC_FILES
  src/main.cpp

  src/joystick_node.cpp

  src/handles/axis.cpp
  src/handles/button.cpp
  src/handles/gamepad_status.cpp
  src/handles/gamepad.cpp
)

set(MAPPINGS_SRC
    src/mappings_main.cpp

    src/mapping_node.cpp
)

# Link the dependencies
add_executable( ${MAIN_EXEC} ${SRC_FILES})
ament_target_dependencies( ${MAIN_EXEC} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Create executable for creating mappings On-The-Fly
add_executable( ${MAPPINGS_EXEC} ${MAPPINGS_SRC} )
ament_target_dependencies( ${MAPPINGS_EXEC} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add Include Directory
target_include_directories( ${MAIN_EXEC} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib>
    $<INSTALL_INTERFACE:lib>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:/home/ubuntu/Documents/Polar-One/ros2_ws/build/joystick_driver/include>
    /opt/ros/jazzy/include
)

target_link_libraries( ${MAIN_EXEC} PRIVATE
    rclcpp::rclcpp
    ${PROJECT_NAME}_parameters
    plib
    yaml-cpp::yaml-cpp
)

target_link_libraries( ${MAPPINGS_EXEC} PRIVATE
    rclcpp::rclcpp
    ${PROJECT_NAME}_parameters
    plib
    yaml-cpp::yaml-cpp
)

# Install Executable
install(TARGETS
  ${MAIN_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)

# Do the same for the mappings exec

# Add Include Directory
target_include_directories( ${MAPPINGS_EXEC} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/lib>
    $<INSTALL_INTERFACE:lib>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    /opt/ros/jazzy/include
)

# Install Executable
install(TARGETS
  ${MAPPINGS_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)

# Bringup (yaml configs, launch files)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  # Add Dependencies
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