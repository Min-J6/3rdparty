#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <memory> // std::make_shared
#include <thread> // std::thread
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class Client {
public:
    // 콜백 변수
    std::function<void(const std::string&)> on_receive;             // 메세지 수신시 호출
    std::function<void(const boost::system::error_code&)> on_error; // 에러 발생시 호출
    std::function<void()> on_connect;                               // 연결 성공시 호출
    std::function<void()> on_disconnect;                            // 연결 종료시 호출


    Client()
        : work_guard_(boost::asio::make_work_guard(io_context_)),
          socket_(io_context_),
          resolver_(io_context_)
    {
        // 쓰레드 생성
        io_thread_ = std::thread([this]() {
            try {
                io_context_.run();
            } catch (std::exception& e) {
                std::cerr << "[Error] [Client] IO 스레드 예외 발생: " << e.what() << std::endl;
            }
        });
    }

    ~Client() {
        close();
        work_guard_.reset();

        if (!io_context_.stopped()) {
            io_context_.stop();
        }

        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

    // 비동기 서버 연결
    void connect(const std::string& host, short port) {
        boost::asio::post(io_context_, [this, host, port]() {
            resolver_.async_resolve(host, std::to_string(port),
                [this](const boost::system::error_code& ec, tcp::resolver::results_type endpoints)
                {
                    if (!ec) {
                        do_connect(endpoints);
                    } else {
                        handle_error(ec, "Resolve");
                    }
                });
        });
    }

    // 비동기 데이터 전송
    void send(const std::string& message) {
        // io_context 스레드에서 쓰기 작업을 수행하도록 post
        boost::asio::post(io_context_, [this, message]() {
            boost::asio::async_write(socket_, boost::asio::buffer(message),
                [this](boost::system::error_code ec, std::size_t /*length*/)
                {
                    if (ec) {
                        handle_error(ec, "쓰기");
                    }
                });
        });
    }

    // 소켓 연결 종료
    void close() {
        boost::asio::post(io_context_, [this]() {
            if (socket_.is_open()) {
                socket_.close();
            }
        });
    }

private:
    // 비동기 연결 시도
    void do_connect(tcp::resolver::results_type endpoints) {
        boost::asio::async_connect(socket_, endpoints,
            [this](const boost::system::error_code& ec, const tcp::endpoint& /*endpoint*/) {
                if (!ec)
                {

                    // 연결 성공
                    if (on_connect)
                        on_connect();
                    else
                        std::cout << "[Info ] [Client] 서버 연결 성공: " << socket_.remote_endpoint() << std::endl;

                    do_read(); // 연결 성공 시 즉시 읽기 시작
                }
                else
                {
                    // 연결 실패
                    handle_error(ec, "Connect");
                }
            });
    }

    // 비동기 데이터 읽기
    void do_read() {
        socket_.async_read_some(boost::asio::buffer(data_, max_length),
            [this](boost::system::error_code ec, std::size_t length)
            {
                if (!ec) {
                    // 읽기 성공
                    if (on_receive) {
                        on_receive(std::string(data_, length));
                    }
                    do_read();
                }
                else if (ec == boost::asio::error::eof) {
                    // 서버가 정상적으로 연결 종료
                    std::cout << "[Info ] [Client] 서버와 연결 종료됨." << std::endl;
                    if (on_disconnect) {
                        on_disconnect();
                    }
                }
                else {
                    // 읽기 오류
                    if (ec.value() != boost::asio::error::operation_aborted) {
                         handle_error(ec, "읽기");
                    }
                }
            });
    }

    // 공용 에러 핸들러
    void handle_error(const boost::system::error_code& ec, const std::string& context) {
        if (on_error) {
            on_error(ec);
        } else {
            std::cerr << "[Error] [Client] " << context << " 오류: " << ec.message() << std::endl;
        }
    }



    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;
    tcp::socket socket_;
    tcp::resolver resolver_;

    enum { max_length = 1024 }; // 서버와 동일하게 설정
    char data_[max_length];
};