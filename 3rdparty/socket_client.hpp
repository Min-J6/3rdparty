#pragma once
#include <boost/asio.hpp>
#include <string>
#include <functional>
#include <thread>

class Client {
public:
    using ReceiveHandler = std::function<void(const std::string&)>;
    using ConnectHandler = std::function<void()>;
    using DisconnectHandler = std::function<void()>;

    Client(const std::string& host, short port);
    ~Client();

    void connect();
    void disconnect();
    void send(const std::string& message);

    // 콜백 등록 함수
    ReceiveHandler on_recveive;
    ConnectHandler on_connect;
    DisconnectHandler on_disconnect;

private:
    void do_receive(); // 내부 비동기 수신 루프

    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket socket_;
    boost::asio::ip::tcp::resolver resolver_;
    boost::asio::streambuf buffer_;

    std::string host_;
    short port_;
    std::thread worker_thread_; // 비동기 처리를 위한 스레드
};



































Client::Client(const std::string& host, short port)
    : socket_(io_context_), resolver_(io_context_), host_(host), port_(port) {}


Client::~Client()
{
    disconnect();
}

void Client::disconnect()
{
    if (!io_context_.stopped())
    {
        io_context_.stop();
    }
    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }
    if (socket_.is_open())
    {
        socket_.close();
    }
}


void Client::connect()
{
    if (io_context_.stopped()) {
        io_context_.restart();
    }

    auto endpoints = resolver_.resolve(host_, std::to_string(port_));
    boost::asio::connect(socket_, endpoints);

    // 연결 성공 콜백 실행
    if (on_connect) {
        on_connect();
    }

    // 비동기 수신 시작
    do_receive();

    // io_context를 별도 스레드에서 실행 (백그라운드 수신)
    worker_thread_ = std::thread([this]()
        {
            io_context_.run();
        });
}


void Client::send(const std::string& message)
{
    boost::asio::write(socket_, boost::asio::buffer(message));
}


void Client::do_receive()
{
    // 개행 문자('\n')를 기준으로 읽는 예시
    boost::asio::async_read_until(socket_, buffer_, '\n',
        [this](boost::system::error_code ec, std::size_t bytes_transferred) {
            if (!ec) {
                std::string data((std::istreambuf_iterator<char>(&buffer_)),
                                 std::istreambuf_iterator<char>());

                // 콜백 함수가 등록되어 있다면 실행
                if (on_recveive) {
                    on_recveive(data);
                }

                do_receive(); // 다음 메시지 수신을 위해 재귀 호출
            } else {
                // 에러 발생 시 (연결 끊김 등) disconnect 콜백 실행
                if (on_disconnect) {
                    on_disconnect();
                }
            }
        });
}