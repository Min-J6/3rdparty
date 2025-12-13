# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)


project(example_imgui)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 서브 모듈
add_subdirectory(imgui)  # <- 추가


# 실행 파일
add_executable(example_imgui
    example_imgui.cpp
    imgui/imgui_demo.cpp # <- 데모를 위해 추가
)


# 라이브러리 링크
target_link_libraries(example_imgui
        imgui           # <- 추가
)