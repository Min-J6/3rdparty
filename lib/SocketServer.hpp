#pragma once
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <boost/asio.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/regex.hpp>

using boost::asio::ip::tcp;

class Session : public std::enable_shared_from_this<Session> {
public:
    std::function<void(std::shared_ptr<Session>, const std::vector<char>&)> on_receive;
    std::function<void(std::shared_ptr<Session>, const boost::system::error_code&)> on_error;


    tcp::socket& socket() {
        return socket_;
    }


    // 서버 시작
    void start() {

        print("[Info ] [Session] 클라이언트 연결됨: " + socket_.remote_endpoint().address().to_string());

        do_read();
    }


    // 비동기 데이터 전송
    void send(const std::string& message) {
        auto self = shared_from_this();
        boost::asio::async_write(socket_, boost::asio::buffer(message),
            [this, self](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (ec) {
                    on_error ? on_error(self, ec) : print("[Error] [Session] 쓰기 오류: " + ec.message());
                }
            });
    }


    // 비동기 데이터 전송
    void send(const std::vector<char>& data) {
        auto self = shared_from_this();
        boost::asio::async_write(socket_, boost::asio::buffer(data),
            [this, self](boost::system::error_code ec, std::size_t /*length*/)
            {
                if (ec) {
                    on_error ? on_error(self, ec) : print("[Error] [Session] 쓰기 오류: " + ec.message());
                }
            });
    }

    std::string ip() {
        return socket_.remote_endpoint().address().to_string();
    }


    // 세션 팩토리 메소드
    static std::shared_ptr<Session> create(boost::asio::io_context& io_context) {
        return std::shared_ptr<Session>(new Session(io_context));
    }


private:
    Session(boost::asio::io_context& io_context)
        : socket_(io_context),
          delimiter_regex_("(\r\n|\r|\n)")
    {}


    // 비동기 데이터 읽기
    void do_read() {
        auto self = shared_from_this();

        // 정규표현식을 구분자로 사용
        boost::asio::async_read_until(socket_, buffer_, delimiter_regex_,
            [this, self](boost::system::error_code ec, std::size_t length)
            {
                if (!ec) {
                    const char* data_ptr = boost::asio::buffer_cast<const char*>(buffer_.data());
                    std::vector<char> bytes_data(data_ptr, data_ptr + length);


                    on_receive ? on_receive(self, bytes_data) : print("[Info ] [Session] 데이터 수신: " + std::string(bytes_data.begin(), bytes_data.end()));


                    buffer_.consume(length);
                    do_read();
                }
                else if (ec == boost::asio::error::eof) {
                    // 정상 종료됨
                    print("[Info ] [Session] 클라이언트 연결 종료: " + socket_.remote_endpoint().address().to_string());
                }
                else {
                    // 에러 발생함
                    on_error ? on_error(self, ec) : print("[Error] [Session] 읽기 오류: " + ec.message());
                }
            });
    }


    // 로그 출력
    void print(const std::string& msg) {
        std::cout << msg << std::endl;
    }



    tcp::socket socket_;
    boost::asio::streambuf buffer_;
    boost::regex delimiter_regex_;
};



class Server {
public:
    std::function<void(std::shared_ptr<Session>)> on_accept;
    std::function<void(const boost::system::error_code&)> on_error;

    Server(short port) : acceptor_(io_context_, tcp::endpoint(tcp::v4(), port)) {

        print("[Info ] [Server] 서버 포트 열림: " + std::to_string(port));

        do_accept();
    }


    // 서버 시작
    void run() {
        io_context_.run();
    }

private:
    // 비동기 클라이언트 연결
    void do_accept() {
        auto new_session = Session::create(io_context_);

        acceptor_.async_accept(new_session->socket(),
            [this, new_session](boost::system::error_code ec)
            {
                if (ec) {
                    // 연결 실패
                    on_error ? on_error(ec) : print("[Error] [Server] Accept 에러: " + ec.message());
                }
                else {
                    // 연결 성공
                    new_session->start();

                    on_accept ? on_accept(new_session) : print("[Info ] [Server] 클라이언트 연결됨: " + new_session->socket().remote_endpoint().address().to_string());
                }

                do_accept();
            });
    }


    // 로그 출력
    void print(const std::string& msg) {
        std::cout << msg << std::endl;
    }

    boost::asio::io_context io_context_;
    tcp::acceptor acceptor_;
};