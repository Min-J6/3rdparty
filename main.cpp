#include <thread>
#include "3rdparty/socket_server.hpp"


void on_receive_callback(std::shared_ptr<SessionInterface> client, const std::string& message)
{
    client->deliver("Server's response: " + message + "\n");
}

void on_connect_callback(std::shared_ptr<SessionInterface> client)
{
    std::cout << "Client connected from " << client->ip() << std::endl;
}

void on_disconnect_callback(std::shared_ptr<SessionInterface> client)
{
    std::cout << "Client disconnected from " << client->ip() << std::endl;
}

int main()
{
    Server server(8080);
    server.on_receive = on_receive_callback;
    server.on_accept = on_connect_callback;
    server.on_disconnect = on_disconnect_callback;

    server.start();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
