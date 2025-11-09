#include "../../lib/SocketServer.hpp"

int main() {
    try
    {
        Server server(12345);

        // 새 클라이언트가 연결될 때마다 호출됨
        server.on_accept = [](std::shared_ptr<Session> session)
        {
            // 환영 메시지 전송
            session->send("서버에 연결되었습니다!\n");

            // 세션 수신 콜백
            session->on_receive = [](std::shared_ptr<Session> s, const std::string& data)
            {
                std::cout << "[Info ] [Session] "
                          << s->socket().remote_endpoint() << ": "
                          << data << std::endl;

                // 에코
                s->send(data);
            };

            // 세션 에러 콜백
            session->on_error = [](std::shared_ptr<Session> s, const boost::system::error_code& ec)
            {
                std::cerr << "[Error] [Session]"
                          << s->socket().remote_endpoint() << "세션 에러: "
                          <<  ec.message() << std::endl;
            };
        };

        server.on_error = [](const boost::system::error_code& ec)
        {
            std::cerr << "[Error] [Server] Accept 에러: " << ec.message() << std::endl;
        };

        server.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << "예외 발생: " << e.what() << std::endl;
    }

    return 0;
}