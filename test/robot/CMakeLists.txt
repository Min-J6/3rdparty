cmake_minimum_required(VERSION 3.10)




# 두산 로봇 IK 예제
add_executable(Test_Robot_DooSan_M1013 doosan_m1013.cpp)




# SIMD 최적화 플래그: Eigen 및 기타 연산을 위해 추가합니다.
target_compile_options(Test_Robot_DooSan_M1013 PRIVATE
    -O3                             # 최고 수준의 최적화
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-march=native> # 현재 CPU에 맞는 최적 명령어셋 사용
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-mavx2>        # AVX2 명령어셋 활성화
)


# 헤더 경로
target_include_directories(Test_Robot_DooSan_M1013 PUBLIC
        ${CMAKE_SOURCE_DIR}/lib
)




# 라이브러리 링크
target_link_libraries(Test_Robot_DooSan_M1013 PUBLIC
        imgui
        Eigen3::Eigen
        ${LAPACK_LIBRARIES}
        ${BLAS_LIBRARIES}
)