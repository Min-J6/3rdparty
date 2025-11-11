#include "../../lib/yaml.hpp"
#include <iostream>

int main()
{
    std::cout << "[테스트] Yaml 실행" << std::endl;

    // ----------------------------------------------
    // Yaml 생성
    //  - 파일이 없으면 새 파일 생성
    // ----------------------------------------------
    Yaml config;
    config.load("config.yaml"); // 파일이 없으므로 '새 파일 생성' 메시지 출력





    // ----------------------------------------------
    // 기본값 설정 ( Default Get and Set )
    //  - '|' 연산자는 파일에 데이터가 없는 경우 기본값으로 설정됨
    //  - 이미 파일에 데이터가 있는 경우 덮어씌어지진 않음
    // ----------------------------------------------

    {
        // 기본 키값
        std::string project_name        = config["project"] | "My Project";

        // .(점)으로 구분된 키
        int port                        = config["server.port"] | 8080;
        std::string host                = config["server.host"] | "127.0.0.1";

        // .(점)으로 구분된 키
        std::string db_type             = config["database.type"]      | "sqlite";
        std::string db_url              = config["database.url"]       | "file:dev.db";
        int db_pool_size                = config["database.pool.size"] | 5;

        // 점과 배열으로 구성된 키
        std::string user1               = config["users[0].name"] | "default_admin";
        int user1_id                    = config["users[0].id"]   | 1;
        std::string user1_tag           = config["users[0].tags"] | "admin" ;

        std::string user2               = config["users[1].name"] | "default_user";
        int user2_id                    = config["users[1].id"]   | 2;
        std::string user2_tag           = config["users[1].tags"] | "user" ;

        // 출력
        std::cout << std::endl;
        std::cout << "  Project Name: " << project_name << "\n" << std::endl;

        std::cout << "Server:" << std::endl;
        std::cout << "  Host: " << host << std::endl;
        std::cout << "  Port: " << port << "\n" << std::endl;

        std::cout << "Database:" << std::endl;
        std::cout << "  Type: " << db_type << std::endl;
        std::cout << "  URL: " << db_url << std::endl;
        std::cout << "  Pool " << std::endl;
        std::cout << "      Size: " << db_pool_size << "\n" << std::endl;

        std::cout << "Users:" << std::endl;
        std::cout << "  - Name: " << user1 << std::endl;
        std::cout << "    ID: " << user1_id << std::endl;
        std::cout << "    Tag: " << user1_tag << std::endl;
        std::cout << "  - Name: " << user2 << std::endl;
        std::cout << "    ID: " << user2_id << std::endl;
        std::cout << "    Tag: " << user2_tag << std::endl;

        std::cout << "============================================" << std::endl;

    }






    // ----------------------------------------------
    // 값 수정하기
    //  - 파일에 데이터가 있으면 '=' 연산자로 덮어 씀
    // ----------------------------------------------
    {
        config["server.port"]        = 1234;
        config["server.host"]        = "0.0.0.0";

        config["database.type"]      = "New DB";
        config["database.url"]       = "file:dev.db";
        config["database.pool.size"] = 999;

        config["users[0].name"]      = "new_admin";
        config["users[0].id"]        = 1;
        config["users[0].tags"]      = "admin" ;

        config["users[1].name"]      = "new_user";
        config["users[1].id"]        = 2;
        config["users[1].tags"]      = "user" ;


        // 기본 키값
        std::string project_name        = config["project"];

        // .(점)으로 구분된 키
        int port                        = config["server.port"];
        std::string host                = config["server.host"];

        // .(점)으로 구분된 키
        std::string db_type             = config["database.type"];
        std::string db_url              = config["database.url"];
        int db_pool_size                = config["database.pool.size"];

        // 점과 배열으로 구성된 키
        std::string user1               = config["users[0].name"];
        int user1_id                    = config["users[0].id"];
        std::string user1_tag           = config["users[0].tags"];

        std::string user2               = config["users[1].name"];
        int user2_id                    = config["users[1].id"];
        std::string user2_tag           = config["users[1].tags"];

        // 출력
        std::cout << std::endl;
        std::cout << "  Project Name: " << project_name << "\n" << std::endl;

        std::cout << "Server:" << std::endl;
        std::cout << "  Host: " << host << std::endl;
        std::cout << "  Port: " << port << "\n" << std::endl;

        std::cout << "Database:" << std::endl;
        std::cout << "  Type: " << db_type << std::endl;
        std::cout << "  URL: " << db_url << std::endl;
        std::cout << "  Pool " << std::endl;
        std::cout << "      Size: " << db_pool_size << "\n" << std::endl;

        std::cout << "Users:" << std::endl;
        std::cout << "  - Name: " << user1 << std::endl;
        std::cout << "    ID: " << user1_id << std::endl;
        std::cout << "    Tag: " << user1_tag << std::endl;
        std::cout << "  - Name: " << user2 << std::endl;
        std::cout << "    ID: " << user2_id << std::endl;
        std::cout << "    Tag: " << user2_tag << std::endl;

        std::cout << "============================================" << std::endl;
    }



    // ----------------------------------------------
    // 리스트 타입
    // ----------------------------------------------
    {
        std::vector<int> list = config["List Type"] | std::vector<int>({ 1, 2, 3, 4, 5 });
        std::vector<std::string> list2 = config["List Type2"] | std::vector<std::string>({ "abc", "defg", "hijk", "lmnop", "qr" });

        // 점과 배열로 구성된 리스트. 아래의 방법은 config["List Type"] | std::vector<int>({ 1, 2, 3, 4, 5 });와 동일한 결과임
        // config["List Type[0]"] = 10;
        // config["List Type[1]"] = 20;
        // config["List Type[2]"] = 30;
        // config["List Type[3]"] = 40;
        // config["List Type[4]"] = 50;
    }


    return 0;
}