project("TracSat_IMU")
cmake_minimum_required(VERSION 3.10)

add_executable(IMU_comms IMU_comms.cpp)

find_package(Boost 1.71.0 REQUIRED COMPONENTS asio serial_port)

target_link_libraries(IMU_comms Boost::asio Boost::serial_port)