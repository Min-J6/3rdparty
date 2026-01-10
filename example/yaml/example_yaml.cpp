#include "yaml.hpp"
#include <iostream>

int main()
{
    // 불러오기
    Yaml config("settings.yaml");

    // 쓰기
    std::string keyname_title = "window.title";
    std::string default_title = "Title name here";
    std::string title = config.get(keyname_title, default_title);

    // 리스트 값 쓰기
    std::string keyName_num = "window.size";
    std::vector<int> default_numlist = std::vector<int> {1920, 1080};
    config.set(keyName_num, default_numlist);

    // 값 읽기
    std::string title_value = config.get<std::string>("window.title");
    std::vector<int> size_value = config.get<std::vector<int>>("window.size");

    // 출력
    std::cout << "title: " << title_value << std::endl;;
    std::cout << "size: " << size_value[0] << ", " << size_value[1] << std::endl;

    return 0;
}