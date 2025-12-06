cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)


project(imgui)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 서브 모듈
add_subdirectory(imgui)


# 실행 파일
add_executable(main
    main.cpp
)


# 라이브러리 링크
target_link_libraries(main
        imgui
)