#include "../../lib/yaml.hpp"
#include <iostream>

int main()
{
    Yaml config;
    config.load("config.yaml"); // 파일이 없으므로 '새 파일 생성' 메시지 출력


    // 기본값 설정 (|=)
    // Yaml 파일에 존재하지 않는 키는 자동으로 생성하고 값을 설정함
    {

        // 단일 타입
        config["server.port"] |= 8080;
        config["server.host"] |= "127.0.0.1";
        config["logging.level"] |= "info"; // 기본은 info 레벨
        config["features.beta"] |= false; // 베타 기능은 기본 비활성화

        // 리스트 타입
        config["default_roles"] |= { "user", "viewer" };

        // .(점)으로 구분된 키
        config["database.type"] |= "sqlite";
        config["database.url"] |= "file:dev.db";
        config["database.pool.size"] |= 5;

        // 점과 배열으로 구성된 키
        config["users[0].name"] |= "default_admin";
        config["users[0].id"] |= 1;
        config["users[0].tags"] |= { "admin" };
    }


    // 덮어 쓰기 [=]
    {
        config["logging.level"] = "debug"; // 'info' -> 'debug' 덮어쓰기
        config["features.beta"] = true; // 'false' -> 'true' 덮어쓰기

        config["database.type"] = "postgresql";
        config["database.url"] = "user:pass@db.host.com/main_db";
        config["database.pool.size"] = 20; // 풀 사이즈 증가

        // 방화벽 IP 목록 업데이트
        config["allowed_ips"] = { "192.168.0.1", "10.0.0.1", "127.0.0.1" };

        // 사용자 목록 업데이트 (기존 목록 덮어쓰기)
        config["users[0].name"] = "Alice"; // default_admin -> Alice
        config["users[0].tags"] = { "admin", "dev" };

        config["users[1].name"] = "Bob"; // 새 사용자 추가
        config["users[1].id"] = 102;
        config["users[1].tags"] = { "user" };
    }


    // 읽기
    {
        // 단일 타입
        int port = config["server.port"];
        std::string host = config["server.host"];
        std::cout << "  [Server] " << host << ":" << port << std::endl;

        // 리스트 타입
        std::vector<std::string> ips = config["allowed_ips"];
        std::cout << "  [Security] " << ips.size() << "개의 허용된 IP 목록 로드." << std::endl;

        // 점과 배열로된 값 읽기
        std::string first_user_name = config["users[0].name"];
        std::vector<std::string> first_user_tags = config["users[0].tags"];
        std::cout << "  [User] 첫 번째 사용자: " << first_user_name << " (Tags: " << first_user_tags[0] << ", ...)" << std::endl;
    }


    return 0;
}