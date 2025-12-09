cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)

project(ImGuiTransparentDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Add lib
add_subdirectory(CycloneDDS)


# idl 파일 빌드
idlc_generate(
        TARGET Ping_idl                              # 생성될 Idl 라이브러리 이름
        FILES "${CMAKE_CURRENT_SOURCE_DIR}/ping.idl" # idl 파일
        WARNINGS no-implicit-extensibility
)


# 실행 파일
add_executable(main_pub
    main_pub.cpp
)

add_executable(main_sub
    main_sub.cpp
)
target_link_libraries(main_sub
        PRIVATE
        CycloneDDS::ddsc
        Ping_idl
)


# 링크
target_link_libraries(main_pub
        PRIVATE
        CycloneDDS::ddsc
        Ping_idl
)