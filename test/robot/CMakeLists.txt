cmake_minimum_required(VERSION 3.10)

# 두산 로봇 IK 예제
add_executable(Test_Robot_DooSan_M1013 doosan_m1013.cpp)




# 헤더 경로
target_include_directories(Test_Robot_DooSan_M1013 PUBLIC
        ${CMAKE_SOURCE_DIR}/lib
)




# 라이브러리 링크
target_link_libraries(Test_Robot_DooSan_M1013 PUBLIC
        imgui
        Eigen3::Eigen
)