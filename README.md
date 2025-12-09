# Imgui
* ImGui 라이브러리를 서브 모듈로 포함하여 빌드합니다.
* 간단한 래핑 클래스로 쉽게 사용할 수 있습니다.


## 프로젝트 구조
```text
workspace/
├── 3rdparty/
│   └── imgui/
├── main.cpp
└── CMakeLists.txt
```

<br>


## 1. 종속성 설치
GLFW3 설치
```shell
  # Install GLFW3
  sudo apt install libglfw3 libglfw3-dev
```

<br>

## 2. 3rdparty 설정
ImGui 라이브러리 클론 (프로젝트 루트에서 실행)
```shell
  # 3rdparty 디렉토리가 생김
  git clone --branch imgui --single-branch https://github.com/Min-J6/3rdparty.git
```

<br>

## 3. CMakeLists.txt 설정
```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)


project(main)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 서브 모듈
add_subdirectory(3rdparty/imgui)  # <- 추가


# 실행 파일
add_executable(main
    main.cpp
)


# 라이브러리 링크
target_link_libraries(main
        imgui           # <- 추가
)
```

<br>

## 예제 코드

### main.cpp
```c++
// main.cpp
#include "imgui.h"
#include <iostream>


int main() 
{
    ImGui::start("데모");

    while (ImGui::isRunning())
    {
        ImGui::context([]()
                {
                    ImGui::Begin("테스트 윈도우");

                    ImGui::Text("abc123");
                    ImGui::Text("ABC456");
                    
                    if (ImGui::Button("버튼")) {
                        std::cout << "Hello?" << std::endl;
                    }
                    ImGui::End();
                });
    }

    ImGui::stop();

    return 0;
}
```