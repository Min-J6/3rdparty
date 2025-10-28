#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <deque> // deque가 여전히 유연성 면에서 좋습니다.
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <memory>
#include <boost/asio.hpp>
#include <dirent.h>
#include <algorithm>
#include <cstring>
#include <cctype>

class SerialPort {
public:
    explicit SerialPort(boost::asio::io_context& io)
        : ioContext_(io), serialPort_(io), readBuffer_(1024), writingInProgress_(false) {}

    ~SerialPort() {
        close();
    }

    std::function<void()> on_open;
    std::function<void()> on_close;
    std::function<void(const std::string&)> on_error;
    std::function<void(const std::vector<char>&)> on_receive;


    bool open(const std::string& portName, unsigned int baudRate) {
        if (is_open())
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

            if (on_open) on_open();
            return true;
        }
        catch (std::exception& e)
        {
            if (on_error) on_error(e.what());
            std::cerr << "[Error] [Serial Port] 포트 열기 실패: " << e.what() << std::endl;
            return false;
        }
    }

    void close() {
        if (!is_open())
            return;

        boost::system::error_code ec;
        serialPort_.cancel(ec);
        serialPort_.close(ec);


        std::scoped_lock lock(writeMutex_);
        writeQueue_.clear();

        if (on_close) on_close();
    }

    bool is_open() const {
        return serialPort_.is_open();
    }

    void send(const std::vector<char>& data) {
        if (!is_open() || data.empty())
            return;

        {
            std::scoped_lock lock(writeMutex_);
            writeQueue_.push_back(data);
        }


        boost::asio::post(ioContext_, [this]() {
            if (!writingInProgress_) {
                do_write();
            }
        });
    }

    void send(const std::string& text) {
        if (text.empty()) return;
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
        DIR* pts_dir = opendir("/dev/pts/");
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
        if (!is_open()) return;

        serialPort_.async_read_some(boost::asio::buffer(readBuffer_),
            [this](const boost::system::error_code& ec, std::size_t bytesTransferred) {

                if (bytesTransferred > 0)
                {
                    std::vector<char> data(readBuffer_.begin(), readBuffer_.begin() + bytesTransferred);
                    if (on_receive) on_receive(data);
                }

                if (!ec)
                {
                    do_read();
                }
                else if (ec != boost::asio::error::operation_aborted)
                {
                    if (on_error)on_error(ec.message());
                    std::cerr << "[Error] [Serial Port] 읽기 실패: " << ec.message() << std::endl;
                    close();
                }
            });
    }

    void do_write() {
        if (!is_open())
            return;


        // 쓰기 작업 시작 플래그 설정
        writingInProgress_ = true;


        // 큐에 있는 데이터들을 버퍼 시퀀스로 교환 (Swap)
        {
            std::scoped_lock lock(writeMutex_);
            activeWriteQueue_.swap(writeQueue_);
        }


        // 전송할 데이터가 없으면 바로 종료
        if (activeWriteQueue_.empty()) {
            writingInProgress_ = false;
            return;
        }


        // 버퍼 시퀀스 생성 (메모리 복사 없음)
        writeBuffers_.clear();
        for (const auto& data : activeWriteQueue_) {
            writeBuffers_.push_back(boost::asio::buffer(data));
        }


        // 분산-수집 I/O를 사용하여 한 번에 모든 데이터를 비동기 전송
        boost::asio::async_write(serialPort_, writeBuffers_,
            [this](const boost::system::error_code& ec, std::size_t /*bytesTransferred*/) {

                // 전송에 사용된 큐는 비워줍니다.
                activeWriteQueue_.clear();

                if (ec)
                {
                    // 에러 발생 시 쓰기 작업 플래그를 설정
                    writingInProgress_ = false;

                    if (on_error) on_error(ec.message());
                    std::cerr << "[Error] [Serial Port] 쓰기 실패: " << ec.message() << std::endl;
                    close();

                    return;
                }

                bool more_to_write = false;
                {
                    std::scoped_lock lock(writeMutex_);
                    more_to_write = !writeQueue_.empty();
                }

                // 쓰기 작업 중에 큐에 새로 추가된 데이터가 있다면 다음 쓰기 시작
                if (more_to_write) {
                    do_write();
                } else {
                    // 더 이상 쓸 데이터가 없으면 플래그를 내림
                    writingInProgress_ = false;
                }
            });
    }

    boost::asio::io_context& ioContext_;
    boost::asio::serial_port serialPort_;
    std::vector<char> readBuffer_;

    std::mutex writeMutex_;
    std::deque<std::vector<char>> writeQueue_; // 데이터가 추가되는 기본 큐 (스레드 안전)
    std::deque<std::vector<char>> activeWriteQueue_; // 실제 전송에 사용될 데이터 큐 (do_write 전용)
    std::vector<boost::asio::const_buffer> writeBuffers_; // 분산-수집 I/O를 위한 버퍼 시퀀스
    bool writingInProgress_; // 현재 쓰기 작업이 진행 중인지 나타내는 플래그
};
