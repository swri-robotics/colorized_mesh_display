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

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Widgets)
add_definitions(-DQT_NO_KEYWORDS)

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
    ${PROJECT_NAME}
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

# QMOC
qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/colorized_mesh_display.h
)

## Colorized Mesh Shape Library
add_library(${PROJECT_NAME}
  src/colorized_mesh_visual.cpp
  ${MOC_FILES}
  src/colorized_mesh_display.cpp
  src/utils.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
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

#############
## Install ##
#############
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
