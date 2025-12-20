cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)

project(ImGuiTransparentDemo)

# 옵션
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math")



add_subdirectory(test)