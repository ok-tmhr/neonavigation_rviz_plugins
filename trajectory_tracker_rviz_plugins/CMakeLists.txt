cmake_minimum_required(VERSION 3.14.4)
project(trajectory_tracker_rviz_plugins)

set(COLCON_DEPENDS
  pluginlib
  trajectory_tracker_msgs
  rviz_common
  rviz_rendering
)

find_package(ament_cmake REQUIRED)
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()


add_compile_options(-std=c++11)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)


  message(STATUS "Using Qt5 based on the QT_VERSION: ${QT_VERSION}")

  qt5_wrap_cpp(MOC_FILES
    include/trajectory_tracker_rviz_plugins/path_with_velocity_display.h
  )

find_file(HAVE_VALIDATE_QUATERNION_H
  NAMES rviz/validate_quaternions.h
)
if(HAVE_VALIDATE_QUATERNION_H)
  add_definitions(-DHAVE_VALIDATE_QUATERNION_H)
endif()


add_library(${PROJECT_NAME}
  src/path_with_velocity_display.cpp
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${COLCON_DEPENDS})


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)