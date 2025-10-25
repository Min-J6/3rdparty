#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <map>

// --- ryml 라이브러리 포함 ---
#include "lib/yaml.hpp"




int main() {
    Yaml config;
    config.load("config.yaml");

    // 값 설정
    config.set("id", 1234);


    // 값 읽기
    std::string host = config.get("database.host", std::string("dev.db.com"));
    int port = config.get("database.port", 5432);
    std::cout << "Host: " << host << "\n";
    std::cout << "Port: " << port << "\n";


    // 리스트 설정
    std::vector<std::string> backup_servers = {"bak1.db.com", "bak2.db.com", "bak3.db.com"};
    config.set("database.backup_servers", backup_servers);

    std::vector<int> enabled_ports = {80, 443, 8080};
    config.set("network.enabled_ports", enabled_ports);


    // 리스트 값 가져오기
    std::vector<std::string> loaded_servers = config.get("database.backup_servers", std::vector<std::string>());
    std::cout << "Backup Servers:\n";
    for (const auto& server : loaded_servers) {
        std::cout << "- " << server << "\n";
    }


    // 파일에 값이 없어 기본값을 사용하는 경우
    std::vector<int> admin_ports = config.get("network.admin_ports", std::vector<int>({9001, 9002}));
    std::cout << "Admin Ports:\n";
    for (int p : admin_ports) {
        std::cout << "- " << p << "\n";
    }


    config.save();

    return 0;
}
