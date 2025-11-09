# -------------------------------------
# 아래 내용을 프로젝트 루트의 CMakeLists.txt에 작성할것..
# -------------------------------------
cmake_minimum_required(VERSION 3.10)
project(TEST_EXAMPLE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 서브 디렉토리 추가
add_subdirectory(lib/CycloneDDS)


# IDL 파일 빌드
idlc_generate(
    TARGET Ping_idl
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/idl/PingData.idl"
    WARNINGS no-implicit-extensibility
)


# Publish 예제
add_executable(test_pulish publish.cpp)
target_link_libraries(test_pulish PUBLIC CycloneDDS::ddsc Ping_idl)
target_include_directories(test_pulish PUBLIC ${CMAKE_CURRENT_BINARY_DIR})


# Subscribe 예제
add_executable(test_subscribe subscrib.cpp)
target_link_libraries(test_subscribe PUBLIC CycloneDDS::ddsc Ping_idl)
target_include_directories(test_subscribe PUBLIC ${CMAKE_CURRENT_BINARY_DIR})