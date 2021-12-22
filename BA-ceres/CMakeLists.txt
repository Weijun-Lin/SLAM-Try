cmake_minimum_required(VERSION 3.0.0)
project(BA-Ceres VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

set(main_src main.cpp DataGenerator.cpp BA.cpp)

# file(GLOB srcs ${CMAKE_SOURCE_DIR}/*.cpp)

add_executable(BA-Ceres ${main_src})
target_link_libraries(BA-Ceres ${OpenCV_LIBS} ${CERES_LIBRARIES} Sophus::Sophus)