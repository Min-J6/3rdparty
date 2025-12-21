cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)

project(main)

# 옵션
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math")


add_subdirectory(3rdparty/lib_ImGui)
add_subdirectory(3rdparty/lib_ddsc)

add_executable(main main.cpp)

idlc_generate(
        TARGET idl_example
        FILES "${CMAKE_CURRENT_SOURCE_DIR}/example.idl" # idl 파일
        WARNINGS no-implicit-extensibility
)

target_link_libraries(main lib_imgui ddsc idl_example)


add_executable(client client.cpp)