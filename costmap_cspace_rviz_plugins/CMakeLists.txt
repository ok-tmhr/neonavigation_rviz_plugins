cmake_minimum_required(VERSION 3.14.4)
project(costmap_cspace_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wshadow)
endif()

set(COLCON_DEPENDS
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  costmap_cspace_msgs
  nav_msgs
)

find_package(ament_cmake_ros REQUIRED)
foreach(COLCON_DEPEND ${COLCON_DEPENDS})
  find_package(${COLCON_DEPEND} REQUIRED)
endforeach()

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall)

set(CMAKE_AUTOMOC ON)

  message(STATUS "Using Qt5 based on the QT_VERSION: ${QT_VERSION}")
  qt5_wrap_cpp(MOC_FILES
    include/costmap_cspace_rviz_plugins/c_space_3d_display.h
  )

add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/c_space_3d_display.cpp
  src/swatch.cpp
  src/palette_builder.cpp
  ${MOC_FILES}
)
ament_target_dependencies(${PROJECT_NAME} ${COLCON_DEPENDS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()