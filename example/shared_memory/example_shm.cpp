#include <atomic>
#include <functional>
#include <thread>
#include <iostream>

#include "shared_memory.hpp"


// 간단한 비동기 타이머
class Timer {
    std::atomic<bool> active{false};
    std::thread worker;

public:
    std::function<void()> callback;

    void start(int interval_ms) {
        if (active) return; // 이미 실행 중이면 무시
        active = true;
        worker = std::thread([this, interval_ms]() {
            while (active) {
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
                if (active && callback) callback();
            }
        });
    }

    void stop() {
        active = false;
        if (worker.joinable()) worker.join();
    }

    ~Timer() { stop(); }
};



// 공유 메모리 예제
int main() {
    std::cout << "[EXAMPLE] Shared Memory\n";

    // 공유 메모리 생성
    std::string shm_name = "/simple_data";
    SharedMemory<int> sm(shm_name);


    // 공유 메모리 쓰기
    Timer timer1;
    timer1.callback = [&]()
    {
        static int num = 0;
        sm.set(++num);
        std::cout << "[Thread 1] num: " << num << std::endl;
    };
    timer1.start(1000); // 1초마다 실행


    // 공유 메모리 읽기
    Timer timer2;
    timer2.callback = [&]()
    {
        int num = sm.get();
        std::cout << "[Thread 2] num: " << num << std::endl;
    };
    timer2.start(200); // 0.5초마다 실행


    // 대기
    std::this_thread::sleep_for(std::chrono::seconds(10));

    timer1.stop();
    timer2.stop();


    // 공유 메모리 해제
    SharedMemory<int>::unlink(shm_name);

    return 0;
}
