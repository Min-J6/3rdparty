#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <memory>
#include <boost/asio.hpp>
#include <dirent.h>
#include <algorithm> // for std::sort
#include <cstring>   // for strncmp, strlen
#include <cctype>    // for isdigit




class SerialPort {
public:
    explicit SerialPort(boost::asio::io_context& io)
        : ioContext_(io), serialPort_(io), readBuffer_(1024) {}

    ~SerialPort() {
        disconnect();
    }

    std::function<void(const std::vector<char>&)> on_receive;
    std::function<void()> on_disconnect;
    std::function<void(const std::string&)> on_error;

    bool connect(const std::string& portName, unsigned int baudRate) {
        if (is_connected())
            return true;

        try
        {
            serialPort_.open(portName);
            serialPort_.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
            serialPort_.set_option(boost::asio::serial_port_base::character_size(8));
            serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serialPort_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            do_read();

            return true;
        }

        catch (std::exception& e)
        {
            std::cerr << "[Error] [Serial Port] 연결 실패: " << e.what() << std::endl;

            if (on_error)
                on_error(e.what());

            return false;
        }
    }

    void disconnect() {
        if (!is_connected())
            return;

        boost::system::error_code ec;

        serialPort_.cancel(ec);
        serialPort_.close(ec);

        if (on_disconnect)
        {
            on_disconnect();
        }
    }

    bool is_connected() const {
        return serialPort_.is_open();
    }

    void send(const std::vector<char>& data) {
        if (!is_connected()) return;

        boost::asio::post(ioContext_, [this, data]() {
            bool write_in_progress = !writeQueue_.empty();

            writeQueue_.push_back(data);

            if (!write_in_progress)
            {
                do_write();
            }
        });
    }

    void send(const std::string& text) {
        send(std::vector<char>(text.begin(), text.end()));
    }

    static std::vector<std::string> getAvailablePorts() {
        std::vector<std::string> ports;

        auto scan_dir = [&](const char* path, const char* prefix[], int count) {
            DIR* dir = opendir(path);
            if (!dir) return;

            struct dirent* entry;
            while ((entry = readdir(dir))) {
                for (int i = 0; i < count; i++) {
                    if (strncmp(entry->d_name, prefix[i], strlen(prefix[i])) == 0) {
                        ports.push_back(std::string(path) + "/" + entry->d_name);
                        break;
                    }
                }
            }
            closedir(dir);
        };

        const char* dev_prefixes[] = {"ttyS", "ttyUSB", "ttyACM"};
        scan_dir("/dev", dev_prefixes, 3);

        // /dev/pts 검색 (숫자만)
        DIR* pts_dir = opendir("/dev/pts");
        if (pts_dir) {
            struct dirent* entry;
            while ((entry = readdir(pts_dir))) {
                if (isdigit(entry->d_name[0])) {
                    ports.push_back(std::string("/dev/pts/") + entry->d_name);
                }
            }
            closedir(pts_dir);
        }

        std::sort(ports.begin(), ports.end());
        return ports;
    }

private:
    void do_read() {
        if (!is_connected()) return;
        serialPort_.async_read_some(boost::asio::buffer(readBuffer_),
            [this](const boost::system::error_code& ec, std::size_t bytesTransferred) {

                if (!ec && bytesTransferred > 0)
                {
                    std::vector<char> data(readBuffer_.begin(), readBuffer_.begin() + bytesTransferred);

                    if (on_receive)
                    {
                        on_receive(data);
                    }

                    do_read();
                }



                if (ec == boost::asio::error::operation_aborted)
                {
                    // 정상 종료
                }
                else if (ec)
                {
                    std::cerr << "[Error] [Serial Port] 읽기 실패: " << ec.message() << std::endl;
                    if (on_error)
                    {
                        on_error(ec.message());
                    }

                    disconnect(); // 에러 발생시 연결 해제
                }
            });
    }

    void do_write() {
        if (!is_connected() || writeQueue_.empty()) return;

        boost::asio::async_write(serialPort_, boost::asio::buffer(writeQueue_.front()),
            [this](const boost::system::error_code& ec, std::size_t /*bytesTransferred*/) {
                if (!ec)
                {
                    writeQueue_.pop_front();

                    if (!writeQueue_.empty())
                    {
                        do_write();
                    }
                }

                if (ec) {
                    std::cerr << "[Error] [Serial Port] 쓰기 실패: " << ec.message() << std::endl;

                    if (on_error)
                    {
                        on_error(ec.message());
                    }

                    disconnect(); // 에러 발생시 연결 해제
                }
            });
    }

    boost::asio::io_context& ioContext_;
    boost::asio::serial_port serialPort_;
    std::vector<char> readBuffer_;
    std::deque<std::vector<char>> writeQueue_;
};