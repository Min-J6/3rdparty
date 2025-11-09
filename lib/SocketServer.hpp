#pragma once
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class Session : public std::enable_shared_from_this<Session> {
public:
    // Public 콜백 변수
    std::function<void(std::shared_ptr<Session>, const std::string&)> on_receive;
    std::function<void(std::shared_ptr<Session>, const boost::system::error_code&)> on_error;

    static std::shared_ptr<Session> create(boost::asio::io_context& io_context) {
        return std::shared_ptr<Session>(new Session(io_context));
    }

    tcp::socket& socket() {
        return socket_;
    }

    void start() {
        std::cout << "[Info ] [Session] 클라이언트 연결됨: "
                  << socket_.remote_endpoint()
                  << std::endl;
        do_read();
    }

    // 클라이언트에게 데이터를 보내는 public 메소드
    void send(const std::string& message) {
        auto self = shared_from_this();

        boost::asio::async_write(socket_, boost::asio::buffer(message),
            [this, self, message](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (ec) {
                    // 데이터 보내다가 에러 발생함
                    if (on_error)
                        on_error(self, ec);
                    else
                        std::cerr << "[Error] [Session] 쓰기 오류: " << ec.message() << std::endl;
                }
            });
    }

private:
    Session(boost::asio::io_context& io_context) : socket_(io_context) {}

    // 비동기 데이터 읽기
    void do_read() {
        auto self = shared_from_this();

        socket_.async_read_some(boost::asio::buffer(data_, max_length),
            [this, self](boost::system::error_code ec, std::size_t length)
            {
                if (!ec) {
                    // 읽은 데이터 콜백 실행
                    if (on_receive) {
                        on_receive(self, std::string(data_, length));
                    }
                    do_read();
                }

                else if (ec == boost::asio::error::eof) {
                    // 정상적으로 연결 종료됨
                    std::cout << "[Info ] [Session] 클라이언트 연결 종료: " << self->socket_.remote_endpoint() << std::endl;
                }

                else {
                    // 에러 발생함
                    if (on_error)
                        on_error(self, ec);
                    else
                        std::cerr << "[Error] [Session] 읽기 오류: " << ec.message() << std::endl;
                }
            });
    }

    tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
};




class Server {
public:
     std::function<void(std::shared_ptr<Session>)> on_accept;           // 클라이언트 연결 시 호출된는 콜백
    std::function<void(const boost::system::error_code&)> on_error;     // 에러 발생 시 호출되는 콜백

    Server(short port) : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port)) {
        std::cout << "[Info ] [Server] 서버 시작, port: "
                  << port << std::endl;
        do_accept();
    }

    void run() {
        io_context_.run();
    }

private:
    void do_accept() {
        auto new_session = Session::create(io_context_);

        acceptor_.async_accept(new_session->socket(),
            [this, new_session](boost::system::error_code ec)
            {
                if (ec)
                {
                    // 클라이언트 연결 실패
                    if (on_error) { on_error(ec); }

                    std::cerr << "[Error] [Server] Accept 오류: "
                            << ec.message() << std::endl;
                }


                // 클라이언트 연결 성공
                new_session->start();

                if (on_accept)
                    on_accept(new_session);
                else
                    std::cout << "[Info ] [Session] 클라이언트 연결됨: " << new_session->socket().remote_endpoint() << std::endl;


                do_accept();
            });
    }

    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;
};