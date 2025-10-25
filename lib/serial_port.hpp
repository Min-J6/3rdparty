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
    SerialPort() : port(io_ctx), heartbeat_timer(io_ctx) {}

    ~SerialPort() {
        disconnect();
    }

    // 콜백 함수 정의
    std::function<void(const std::vector<char>&)> on_receive;
    std::function<void()> on_disconnect;
    std::function<void(const std::string&)> on_error;


    // -------------------------------------------------
    // 연결
    // -------------------------------------------------

    // 연결
    bool connect(const std::string& port_path, int baud_rate) {
        if (connected) disconnect();

        boost::system::error_code ec;
        port.open(port_path, ec);
        if (ec) {
            std::string error_msg = "시리얼 포트 연결 실패: " + ec.message();
            std::cout <<  "[Error] [Serial Port] " << error_msg << std::endl;
            if (on_error) {
                on_error(error_msg); // 에러 콜백 호출
            }
            return false;
        }

        port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        port.set_option(boost::asio::serial_port_base::character_size(8));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        connected = true;
        start_receive();

        // io_thread가 없거나 종료된 경우에만 새로 시작
        if (!io_thread.joinable()) {
            io_thread = std::thread([this]() { io_ctx.run(); });
        }

        std::cout << "[Info ] [Serial Port] 시리얼 포트 연결: " << port_path << std::endl;

        return true;
    }

    // 연결 해제
    void disconnect() {
        if (!connected) return;

        connected = false;
        disable_heartbeat();

        // 포트 닫기
        if (port.is_open()) {
            boost::system::error_code ec;
            port.close(ec);
        }

        // io_context 정지
        io_ctx.stop();

        // io_thread가 현재 쓰레드가 아닐 때만 join
        if (io_thread.joinable()) {
            if (io_thread.get_id() != std::this_thread::get_id()) {
                io_thread.join();
            } else {
                // 현재 쓰레드가 io_thread인 경우 detach
                io_thread.detach();
            }
        }

        // 송신 큐 비우기
        std::queue<std::vector<char>> empty_queue;
        tx_queue.swap(empty_queue);

        // io_context 재시작 준비 (중요!)
        io_ctx.restart();

        // 콜백 호출
        if (on_disconnect) {
            on_disconnect();
        }

        std::cout << "[Info ] [Serial Port] 연결 해제" << std::endl;
    }

    // 연결 상태 확인
    bool is_connected() const {
        return connected;
    }


    // -------------------------------------------------
    // 통신
    // -------------------------------------------------

    // 송신 바이트 배열
    void send(const std::vector<char>& data) {
        if (!connected) return;

        boost::asio::post(io_ctx, [this, data]() {
            bool write_in_progress = !tx_queue.empty();
            tx_queue.push(data);

            if (!write_in_progress) {
                do_write();
            }
        });
    }

    // 송신 문자열
    void send(const std::string& data) {
        send(std::vector<char>(data.begin(), data.end()));
    }



    // -------------------------------------------------
    // 이벤트 콜백 등록
    // -------------------------------------------------

    // 수신 콜백 등록
    void set_receive_callback(std::function<void(const std::vector<char>&)> callback) {
        on_receive = callback;
    }

    // 연결 종료 콜백 등록
    void set_disconnect_callback(std::function<void()> callback) {
        on_disconnect = callback;
    }

    // 에러 콜백 등록
    void set_error_callback(std::function<void(const std::string&)> callback) {
        on_error = callback;
    }


    // -------------------------------------------------
    // 하트비트
    // -------------------------------------------------
    void enable_heartbeat(const std::vector<char>& data, int interval_ms) {
        boost::asio::post(io_ctx, [this, data, interval_ms]() {
            hb_data = data;
            hb_interval = interval_ms;
            hb_enabled = true;
            schedule_hb();
        });
    }

    // 하트비트 활성화
    void enable_heartbeat(const std::string& data, int interval_ms) {
        enable_heartbeat(std::vector<char>(data.begin(), data.end()), interval_ms);
    }

    // 하트비트 비활성화
    void disable_heartbeat() {
        boost::asio::post(io_ctx, [this]() {
            hb_enabled = false;
            boost::system::error_code ec;
            heartbeat_timer.cancel(ec);
        });
    }


    // 하트비트 상태 확인
    bool is_heartbeat_enabled() const {
        return hb_enabled;
    }

    // 사용 가능한 포트 검색
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
    // Boost.Asio 객체
    boost::asio::io_context io_ctx;
    boost::asio::serial_port port;
    boost::asio::steady_timer heartbeat_timer;
    std::thread io_thread;

    // 상태
    std::atomic<bool> connected{false};

    // 송신 큐
    std::queue<std::vector<char>> tx_queue;

    // 수신 버퍼
    std::array<char, 1024> rx_buffer;

    // 하트비트
    std::atomic<bool> hb_enabled{false};
    std::vector<char> hb_data;
    int hb_interval;

    // 수신 처리 (비동기)
    void start_receive() {
        port.async_read_some(boost::asio::buffer(rx_buffer),
            [this](const boost::system::error_code& ec, std::size_t len) {
                if (!ec && connected) {
                    if (on_receive) {
                        on_receive(std::vector<char>(rx_buffer.begin(), rx_buffer.begin() + len));
                    }
                    start_receive();
                } else if (ec && connected) {
                    std::string error_msg = "수신 에러: " + ec.message();
                    std::cerr << "[Warn ] [Serial Port] " << error_msg << std::endl;
                    if (on_error) {
                        on_error(error_msg); // 에러 콜백 호출
                    }

                    // 별도 쓰레드에서 disconnect 호출 (데드락 방지)
                    std::thread([this]() {
                        disconnect();
                        if (on_error) on_error("연결 끊김");
                    }).detach();
                }
            });
    }

    // 송신 처리 (비동기)
    void do_write() {
        if (tx_queue.empty() || !connected) return;

        auto data_ptr = std::make_shared<std::vector<char>>(tx_queue.front());

        boost::asio::async_write(port, boost::asio::buffer(*data_ptr),
            [this, data_ptr](const boost::system::error_code& ec, std::size_t) {
                if (ec) {
                    std::string error_msg = "송신 에러: " + ec.message();
                    std::cerr << "[Error] [Serial Port] " << error_msg << std::endl;
                    if (on_error) {
                        on_error(error_msg); // 에러 콜백 호출
                    }

                    if (ec == boost::system::errc::io_error || ec == boost::asio::error::operation_aborted) {
                        // 별도 쓰레드에서 disconnect 호출 (데드락 방지)
                        std::thread([this]() {
                            disconnect();
                            if (on_error) on_error("연결 끊김");
                        }).detach();
                        return;
                    }
                }


                if (!tx_queue.empty()) {
                    tx_queue.pop();
                }

                if (!tx_queue.empty() && connected) {
                    do_write();
                }
            });
    }


    // 하트비트 스케줄링 (비동기)
    void schedule_hb() {
        if (!hb_enabled || !connected) return;

        heartbeat_timer.expires_after(std::chrono::milliseconds(hb_interval));
        heartbeat_timer.async_wait([this](const boost::system::error_code& ec) {
            if (!ec && hb_enabled && connected) {
                send(hb_data);
                schedule_hb();
            }
        });
    }
};
