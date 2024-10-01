cmake_minimum_required(VERSION 3.8)
project(slam_debug)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(GTSAM REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM ${EIGEN3_INCLUDE_DIR}
)

########################################
# VILENS SLAM ROS node
add_executable(simple_slam src/simple_slam/simple_slam.cpp src/simple_slam/sission_data.cpp)
target_link_libraries(simple_slam gtsam)


#############
## Install ##
#############

#install(PROGRAMS
#  python/slam_debug/delay_camera.py
#  DESTINATION lib/${PROJECT_NAME}
#)

## Mark libraries for installation
#install(TARGETS
#  DESTINATION lib
#)

install(TARGETS 
  simple_slam
  DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")

#install(
#  DIRECTORY config launch processing_scripts
#  DESTINATION share/${PROJECT_NAME}
#)

#install(
#  DIRECTORY config launch processing_scripts
#  DESTINATION share/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*procman.pmd"
#                 PATTERN "*.yaml"
#                 PATTERN "*.xml"
#                 PATTERN "*.perspective"
#                 PATTERN "*.rviz"
#                 PATTERN "*.py"
#                 PATTERN "*.launch"
#)


