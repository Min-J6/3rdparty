#pragma once
#include <iostream>
#include <string>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/regex.hpp>

using boost::asio::ip::tcp;

class Client {
public:
    std::function<void(const std::vector<char>&)> on_receive;
    std::function<void(const boost::system::error_code&)> on_error;
    std::function<void()> on_connect;
    std::function<void()> on_disconnect;


    Client()
        : work_guard_(boost::asio::make_work_guard(io_context_)),
          socket_(io_context_),
          resolver_(io_context_),
          delimiter_regex_("(\r\n|\r|\n)")
    {
        io_thread_ = std::thread([this]() {
            try {
                io_context_.run();
            } catch (std::exception& e) {
                backlog("[Client] IO 스레드 예외 발생: " + std::string(e.what()));
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


    // 비동기 서버에 연결
    void connect(const std::string& host, short port) {
        boost::asio::post(io_context_, [this, host, port]() {
            resolver_.async_resolve(host, std::to_string(port),
                [this](const boost::system::error_code& ec, tcp::resolver::results_type endpoints)
                {
                    if (!ec) {
                        // 연결 시도
                        do_connect(endpoints);
                    } else {
                        on_error ? on_error(ec)
                        : backlog("[Client] 서버 연결 실패: " + ec.message());
                    }
                });
        });
    }


    // 비동기 서버에 전송
    void send(const std::string& message) {
        boost::asio::post(io_context_, [this, message]() {
            boost::asio::async_write(socket_, boost::asio::buffer(message),
                [this](boost::system::error_code ec, std::size_t /*length*/)
                {
                    if (ec) {
                        on_error ? on_error(ec)
                        : backlog("[Client] 전송 실패: " + ec.message());
                    }
                });
        });
    }


    // 비동기 서버에 전송
    void send(const std::vector<char>& data) {
        boost::asio::post(io_context_, [this, data]() {
            boost::asio::async_write(socket_, boost::asio::buffer(data),
                [this](boost::system::error_code ec, std::size_t /*length*/)
                {
                    if (ec) {
                        on_error ? on_error(ec)
                        : backlog("[Client] 전송 실패: " + ec.message());
                    }
                });
        });
    }


    // 서버 연결 종료
    void close() {
        boost::asio::post(io_context_, [this]() {
            if (socket_.is_open()) {
                socket_.close();
            }
        });
    }


private:
    // 비동기 서버 연결
    void do_connect(tcp::resolver::results_type endpoints) {
        boost::asio::async_connect(socket_, endpoints,
            [this](const boost::system::error_code& ec, const tcp::endpoint& /*endpoint*/) {
                if (!ec) {
                    // 서버 연결 성공

                    on_connect ? on_connect()
                    : backlog("[Client] 서버와 연결됨:" + socket_.remote_endpoint().address().to_string());

                    do_read();
                } else {
                    // 서버 연결 실패
                    backlog("[Client] 서버 연결 실패: " + ec.message());
                }
            });
    }


    // 비동기 데이터 읽기
    void do_read() {
        boost::asio::async_read_until(socket_, buffer_, delimiter_regex_,
            [this](boost::system::error_code ec, std::size_t length)
            {
                if (!ec) {
                    // 메세지 읽기 성공

                    const char* data_ptr = boost::asio::buffer_cast<const char*>(buffer_.data());
                    std::vector<char> bytes_data(data_ptr, data_ptr + length);

                    on_receive ? on_receive(bytes_data)
                    : backlog("[Client] 데이터 수신: " + std::string(bytes_data.begin(), bytes_data.end()));

                    buffer_.consume(length);
                    do_read();
                }
                else if (ec == boost::asio::error::eof) {
                    // 정상 종료됨
                    on_disconnect ? on_disconnect()
                    : backlog("[Client] 서버와 연결 종료됨");
                }
                else if (ec.value() != boost::asio::error::operation_aborted) {
                    // 에러 발생함
                    on_error ? on_error(ec)
                    : backlog("[Client] 읽기 실패: " + ec.message());
                }
            });
    }

    // 로그 출력
    void backlog(const std::string& message) {
        std::cout << "[BackLog] " << message << std::endl;
    }

    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;
    tcp::socket socket_;
    tcp::resolver resolver_;
    boost::asio::streambuf buffer_;
    boost::regex delimiter_regex_;
};