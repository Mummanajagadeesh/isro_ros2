cmake_minimum_required(VERSION 3.8)
project(slam_on_vid)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

set(ORB_SLAM3_DIR ${PROJECT_SOURCE_DIR}/../ORB_SLAM3)

message(${ORB_SLAM3_DIR})

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/g2o
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
)

add_executable(slam_on_vid src/main.cc)

target_link_libraries(slam_on_vid
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
)

# If you encounter Eigen deprecated warnings, you can disable them
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wno-deprecated-declarations)
elseif(MSVC)
    add_compile_options(/wd4996)
endif()

