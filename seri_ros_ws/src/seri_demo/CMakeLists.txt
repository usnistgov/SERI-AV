cmake_minimum_required(VERSION 3.8)
project(seri_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)

#-----------------------------
# Python
#-----------------------------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  script/demo_action.py
  script/vehicle_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
