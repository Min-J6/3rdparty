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
