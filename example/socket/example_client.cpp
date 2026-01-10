#include "socket_client.hpp"
#include <iostream>

void recived_callback(const std::string& message)
{
        std::cout << "[Client] 서버 메세지: " << message << std::endl;
}

void connect_callback()
{
    std::cout << "[Client] 서버에 연결됨\n";
}

void disconnect_callback()
{
    std::cout << "[Client] 서버와 연결 해제됨\n";
}


int main()
{
    Client client = Client("127.0.0.1", 1234);

    client.on_recveive = recived_callback;
    client.on_connect = connect_callback;
    client.on_disconnect = disconnect_callback;
    client.connect();

    std::string message;
    while (std::getline(std::cin, message))
    {
        if (message == "exit")
        {
            break;
        }
        client.send(message + "\n");
    }

    return 0;
}