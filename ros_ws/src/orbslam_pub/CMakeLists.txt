cmake_minimum_required(VERSION 3.8)
project(orbslam_pub)

set(ORB_SLAM3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../ORB_SLAM3")

set(CMAKE_INSTALL_RPATH "${ORB_SLAM3_DIR}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# to remove deprecation errors
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wno-deprecated-declarations)
elseif(MSVC)
    add_compile_options(/wd4996)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization) 

include_directories(
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/Thirdparty
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/include 
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
)

add_executable(getpose src/orbslam_getpose.cpp)

target_include_directories(getpose PRIVATE
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(getpose
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
)

ament_target_dependencies(getpose 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

install(TARGETS
  getpose
  DESTINATION lib/${PROJECT_NAME})

ament_package()
