cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)


project(imgui)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 라이브러리
find_package(Boost REQUIRED COMPONENTS system regex)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# 서브 모듈
add_subdirectory(imgui)  # 추가


# 실행 파일
add_executable(main
    main.cpp
)


# 라이브러리 링크
target_link_libraries(main
        imgui           # 추가
        ${Boost_LIBRARIES}
)