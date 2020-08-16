cmake_minimum_required(VERSION 3.17)
project(DroneTest)

set(CMAKE_CXX_STANDARD 20)
add_executable(DroneTest src/main.cpp src/Drone/DroneParams.h src/Drone/DroneModel.cpp src/Drone/DroneModel.h)
link_libraries(eigen)