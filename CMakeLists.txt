cmake_minimum_required(VERSION 2.8.3)
project(colorized_mesh_display)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  shape_msgs
  std_msgs
  rviz
  pcl_ros
)

add_message_files(
  FILES
    ColorizedMesh.msg
    ColorizedMeshStamped.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    shape_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    colorized_mesh_visual
    ${PROJECT_NAME}
    utils
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    shape_msgs
    std_msgs
    rviz
    pcl_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# QT
add_definitions(-DQT_NO_KEYWORDS)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# QMOC
qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/colorized_mesh_display.h
)

qt5_wrap_cpp(TEST_MOC_FILES
  include/${PROJECT_NAME}/test_display.h
)

## Colorized Mesh Shape Library
add_library(colorized_mesh_visual
  src/colorized_mesh_visual.cpp
)
add_dependencies(colorized_mesh_visual
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(colorized_mesh_visual
  ${catkin_LIBRARIES}
)

# Colorized Mesh Display Library
add_library(${PROJECT_NAME}
  src/colorized_mesh_display.cpp
  ${MOC_FILES}
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  colorized_mesh_visual
)

# Conversion Utilities Library
add_library(utils
  src/utils.cpp
)
add_dependencies(utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(utils
  ${catkin_LIBRARIES}
)

# Test Node
add_executable(${PROJECT_NAME}_test_node
  src/colorized_mesh_display_test.cpp
)
add_dependencies(${PROJECT_NAME}_test_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_test_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
