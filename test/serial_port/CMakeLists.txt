cmake_minimum_required(VERSION 3.10)

# Serial Port  테스트
add_executable(Test_SerialPort main.cpp)

target_include_directories(Test_SerialPort PUBLIC
        ${CMAKE_SOURCE_DIR}/lib
)