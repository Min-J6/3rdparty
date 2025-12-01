cmake_minimum_required(VERSION 3.10)
project(ImGuiTransparentDemo)

# CMake 정책 경고(CMP0167)를 제거하고 최신 Boost 설정 방식을 따릅니다.
cmake_policy(SET CMP0167 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math")

find_package(Boost REQUIRED COMPONENTS system regex)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(lib/imgui)
add_subdirectory(lib/CycloneDDS)

add_executable(main
    main.cpp
)

# 🚀 SIMD 최적화 플래그: Eigen 및 기타 연산을 위해 추가합니다.
#target_compile_options(main PRIVATE
#    -O3                             # 최고 수준의 최적화
#    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-march=native> # 현재 CPU에 맞는 최적 명령어셋 사용
#    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>>:-mavx2>        # AVX2 명령어셋 활성화
#)


target_link_libraries(main PRIVATE imgui Eigen3::Eigen)


find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)

target_link_libraries(main PRIVATE ${LAPACK_LIBRARIES})
target_link_libraries(main PRIVATE ${BLAS_LIBRARIES})


add_subdirectory(test)