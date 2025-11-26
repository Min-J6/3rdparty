cmake_minimum_required(VERSION 3.10)

add_executable(Test_Web_Robot main.cpp)


# 라이브러리 경로
target_include_directories(Test_Web_Robot
        PUBLIC
            ${CMAKE_SOURCE_DIR}/lib
)

target_link_libraries(Test_Web_Robot
    PUBLIC
        imgui
        Eigen3::Eigen

)

# 실행 파일(.exe, .out)의 출력 디렉토리 설정
set_target_properties(Test_Web_Robot
        PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)