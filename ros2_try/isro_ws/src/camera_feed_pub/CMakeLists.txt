cmake_minimum_required(VERSION 3.8)
project(camera_feed_pub)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(camera_feed_pub src/camera_feed_pub.cpp)
ament_target_dependencies(camera_feed_pub rclcpp image_transport sensor_msgs cv_bridge)

target_link_libraries(camera_feed_pub ${OpenCV_LIBS})

install(TARGETS camera_feed_pub DESTINATION lib/${PROJECT_NAME})

ament_package()
