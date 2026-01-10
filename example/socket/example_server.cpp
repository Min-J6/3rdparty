#include "socket_server.hpp"
#include <iostream>
#include <thread>

void recived_callback(const ClinetHandler& client, const std::string& message)
{
    client->deliver("echo " + message + "\n");
    std::cout << "[Server] 클라이언트 메세지: " << message << std::endl;
}

void accept_callback(const ClinetHandler& client)
{
    std::cout << "[Server] 클라이언트 연결됨: " << client->ip() << std::endl;
}

void disconnet_callback(const ClinetHandler& client)
{
    std::cout << "[Server] 클라이언트 연결 해제됨: " << client->ip() << std::endl;
}

int main()
{
    int port = 1234;
    Server server(port);

    server.on_receive = recived_callback;
    server.on_accept = accept_callback;
    server.on_disconnect = disconnet_callback;

    std::thread([&]()
    {
        server.run();
    }).detach();


    std::cout << "(Enter 입력 시 종료)\n";
    std::cin.get();

    server.stop();

    return 0;
}
