cmake_minimum_required(VERSION 3.8)
project(laserline)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
    Boost
    ament_cmake
    rclcpp
    geometry_msgs
    sensor_msgs
    visualization_msgs
)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

add_executable(laserline src/main.cpp src/laser_feature_ros.cpp src/line_feature.cpp)
target_include_directories(laserline PUBLIC include)
ament_target_dependencies(laserline ${dependencies})

install(TARGETS laserline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
 laserline
 DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(laserline)
ament_export_dependencies(${dependencies})

ament_package()

