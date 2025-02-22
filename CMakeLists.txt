cmake_minimum_required(VERSION 3.8)
project(laserline)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(wall_line_detection_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(nlohmann_json REQUIRED)

set(dependencies
    Boost
    ament_cmake
    rclcpp
    geometry_msgs
    sensor_msgs
    visualization_msgs
    std_msgs
    wall_line_detection_msgs
    nav_msgs
    rclcpp_components
    tf2_geometry_msgs
    tf2
    tf2_ros
    angles
    nlohmann_json
)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

# laserline
add_executable(laserline src/main.cpp src/laser_feature_ros.cpp src/line_feature.cpp)
target_include_directories(laserline PUBLIC include)
ament_target_dependencies(laserline ${dependencies})

#  line_path_compare
add_library(line_path_compare_lib SHARED src/line_path_compare.cpp)
target_include_directories(line_path_compare_lib PUBLIC include)
ament_target_dependencies(line_path_compare_lib
  ${dependencies}
)
rclcpp_components_register_node(line_path_compare_lib
    PLUGIN "line_path_compare::LinePathCompare"
    EXECUTABLE line_path_compare)

# test line_path_compare
add_executable(test_compare src/test_compare.cpp src/line_path_compare.cpp)
target_include_directories(test_compare PUBLIC include)
ament_target_dependencies(test_compare ${dependencies})

install(TARGETS 
  laserline 
  line_path_compare_lib
  test_compare
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
 laserline line_path_compare_lib test_compare
 DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(laserline line_path_compare_lib test_compare)
ament_export_dependencies(${dependencies})

ament_package()

