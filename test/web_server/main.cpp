#include "web_server.hpp"
#include <iostream>



// 전역 세션 관리
std::mutex g_sessions_mutex;
std::set<std::shared_ptr<WebServer::session>> g_sessions;


void connect_callback(std::shared_ptr<WebServer::session> session)
{
    {
        std::lock_guard<std::mutex> lock(g_sessions_mutex);
        g_sessions.insert(session);
    }
    session->send("Connected!");
}


void disconnect_callback(std::shared_ptr<WebServer::session> session)
{
    {
        std::lock_guard<std::mutex> lock(g_sessions_mutex);
        g_sessions.erase(session);
    }
    session->send("Disconnected!");
}

void receive_callback(std::shared_ptr<WebServer::session> session, std::string message)
{
    session->send("You sent: " + message);
}


// =================================================================
//              메인 함수 (콜백 로직 수정됨)
// =================================================================
int main(int argc, char* argv[])
{
    auto const address = WebServer::make_address("127.0.0.1");
    auto const port = static_cast<unsigned short>(8080);
    auto const threads = std::max<int>(1, static_cast<int>(std::thread::hardware_concurrency()));

    WebServer::net::io_context ioc{threads};
    auto listener_ptr = std::make_shared<WebServer::listener>(ioc, WebServer::tcp::endpoint{address, port});

    // --- 여기에 콜백 함수들을 정의하고 리스너에 주입합니다 ---
    listener_ptr->on_session_created = [](std::shared_ptr<WebServer::session> s)
    {
        std::cout << "[Main] New session created. Setting callbacks." << std::endl;

        s->on_connect = connect_callback;

        s->on_receive = receive_callback;

        s->on_disconnect = disconnect_callback;
    };


    std::thread t_sender = std::thread([&]()
    {
        while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            std::cout << "Sending stream..." << std::endl;

            // 모든 활성 세션에게 메시지 전송
            std::lock_guard<std::mutex> lock(g_sessions_mutex);
            for(auto& s : g_sessions)
            {
                s->send("Sending stream...");
            }
        }
    });

    t_sender.detach();


    listener_ptr->run();
    std::cout << "Server listening on 127.0.0.1:" << port << std::endl;

    std::vector<std::thread> v;
    v.reserve(threads - 1);
    for(auto i = threads - 1; i > 0; --i)
        v.emplace_back([&ioc] { ioc.run(); });
    ioc.run();

    return EXIT_SUCCESS;
}

