cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules (robot_bringup/robot_bringup/*.py)
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts directory (if you add any later)
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install maps directory
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz directory
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
