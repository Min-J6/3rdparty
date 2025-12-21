#include <vector>
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
