cmake_minimum_required(VERSION 3.0.0)
project(slam_try VERSION 0.1.0)

include(CTest)
enable_testing()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

add_executable(ceres_test ceres_test.cpp)
target_link_libraries(ceres_test ${OpenCV_LIBS} ${CERES_LIBRARIES})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
