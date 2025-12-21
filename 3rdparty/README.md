# 라이브러리 사용법

## 목차

* [SharedMemory](#shared-memory)
* [YAML](#yaml)

<br>

## Shared Memory
* 프로세스간 데이터 공유를 할 수 있습니다.
* SharedMemory::unlink()를 호출하기 전까지 메모리가 유지됩니다.

```c++
#include "shared_mem.hpp"
#include <iostream>


struct MyData {
    int count;
};

int main() 
{
    // 생성
    SharedMemory<MyData> shm("AutoStorage");
    
    
    // 쓰기
    MyData data;
    data.count = 0;
    shm.set(data);

    
    // 읽기
    MyData& data = shm.get();
    data.count++;

    
    std::cout << "현재 카운트: " << data.count << std::endl;

    return 0;
}
```

<br>


## YAML
* 간단한 yaml 파싱 및 설정 기능을 제공합니다.

```c++
#include "3rdparty/yaml.hpp"

int main() 
{
    Yaml config;
    config.load("settings.yaml");

    // 1. 값 읽기 (없으면 기본값으로 파일에 저장됨)
    int width = config.get("window.size.width", 1920);
    std::string title = config.get("window.title", std::string("My Game"));

    // 2. 리스트 읽기
    std::vector<int> scores = config.get("player.scores", std::vector<int>({10, 20, 30}));

    // 3. 값 쓰기
    config.set("network.ip", "127.0.0.1");
    config.set("window.size.height", 1080);

    
    return 0;
}
```