cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)

project(main)

# 옵션
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math") # Eigen 최적화



if (PROJECT_IS_TOP_LEVEL)
    add_executable(example_shared_memory example/shared_memory/example_shm.cpp) # shared mem
    target_include_directories(example_shared_memory PRIVATE 3rdparty)          # shared mem
    add_executable(example_yaml example/yaml/example_yaml.cpp)  # yaml
    target_include_directories(example_yaml PRIVATE 3rdparty)   # yaml
endif ()