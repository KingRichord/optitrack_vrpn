cmake_minimum_required(VERSION 2.8.3)
project(vrpn_client_ros)


# set the C++ standard to C++ 11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads)
include_directories(vrpn_install/include/)
link_directories(vrpn_install/lib/)
add_executable(optitrack_node
        main.cpp
        optitrack.cpp)
target_link_libraries(optitrack_node
        gpsnmea
        vrpn
        vrpnserver
        quat
        vrpn_atmel)



