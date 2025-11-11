cmake_minimum_required(VERSION 3.10)

# IDL 파일 빌드
idlc_generate(
    TARGET Ping_idl                                  # 생성될 Idl 라이브러리 이름
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/PingData.idl" # idl 파일
    WARNINGS no-implicit-extensibility
)


# Publish 예제
add_executable(Test_Publish publish.cpp)
target_link_libraries(Test_Publish
    PUBLIC
        CycloneDDS::ddsc # 외부 라이브러리 링크
        Ping_idl         # IDL 생성 코드 링크
)
target_include_directories(Test_Publish PUBLIC ${CMAKE_CURRENT_BINARY_DIR})


# Subscribe 예제
add_executable(Test_Subscribe subscrib.cpp)
target_link_libraries(Test_Subscribe
        PUBLIC
        CycloneDDS::ddsc # 외부 라이브러리 링크
        Ping_idl         # IDL 생성 코드 링크
)
target_include_directories(Test_Subscribe PUBLIC ${CMAKE_CURRENT_BINARY_DIR})