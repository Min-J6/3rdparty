# Imgui 서브 모듈


```shell
  # Clone
  git clone --branch imgui --single-branch https://github.com/Min-J6/lib
```
<br>

```shell
  # Install GLFW3
  sudo apt install libglfw3 libglfw3-dev
```

<br>

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)


project(imgui)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 서브 모듈
add_subdirectory(lib/imgui)  # <- 추가


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

```c++
// main.cpp
#include "imgui.h"
#include <iostream>


int main() {

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

    std::cout << "Hello World!" << std::endl;

    return 0;
}
```