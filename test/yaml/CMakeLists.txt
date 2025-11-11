cmake_minimum_required(VERSION 3.10)

# Yaml 테스트
add_executable(Test_Yaml main.cpp)


# 라이브러리 경로
target_include_directories(Test_Yaml
        PUBLIC
            ${CMAKE_SOURCE_DIR}/lib
)


# 실행 파일(.exe, .out)의 출력 디렉토리 설정
set_target_properties(Test_Yaml
        PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)