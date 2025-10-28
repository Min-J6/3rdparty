# YAML 파싱 모듈
Rapid YAML 라이브러리를 포팅한 파싱 모듈


## 특징
* Header-Only
* 리스트 벡터 타입 지원
* 간단한 포팅 인터페이스


## 주의
* 읽어온 yaml 파일에 노드가 없는 키값을 읽을 경우 에러가 발생
``` text
[Info ] [Yaml] 새 파일 생성: config.yaml
..lib/yaml.hpp:22752: check failed: (m_cap > 0 && m_size > 0)
```
<br>

---

### 사용법:
``` c++
#include "lib/yaml.hpp"
#include <iostream>

int main() {
    Yaml yaml;

    yaml.load("config.yaml");

    // 쓰기
    yaml["server.host"] = "localhost";
    yaml["server.port"] = 8080;
    yaml["users"] = {"alice", "bob", "carol"};
    yaml["projects[0].name"] = "Alpha";
    yaml["projects[0].lang"] = "C++";
    yaml["projects[1].name"] = "Beta";
    yaml["projects[1].lang"] = "Python";

    // 읽기
    std::string host = yaml["server.host"];
    double port = yaml["server.port"];
    std::vector<std::string> users = yaml["users"];

    std::cout << "Host: " << host << "\n";
    std::cout << "Port: " << port << "\n";
    std::cout << "Users:\n";
    for (auto& u : users)
        std::cout << "  - " << u << "\n";

    yaml.save();
}
```

### 출력:
```text
[Info ] [Yaml] 새 파일 생성: config.yaml
Host: localhost
Port: 8080
Users:
  - alice
  - bob
  - carol
```