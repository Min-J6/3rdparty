#pragma once
#include <iostream>
#include <queue>
#include <mutex>
#include <functional>
#include <utility>

class TaskQueue {
public:
    TaskQueue() : running(true) {}

    TaskQueue(const TaskQueue&) = delete;
    TaskQueue& operator=(const TaskQueue&) = delete;

    // 다른 스레드에서 작업을 추가할 때 호출
    template <typename F>
    void post(F&& f) {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (!running) return;
        tasks.emplace(std::forward<F>(f));
    }

    void stop() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        running = false;
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(queue_mutex);
        return tasks.size();
    }

    bool run_one() {
        std::function<void()> task;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);

            if (tasks.empty()) {
                return false;   // 작업이 없음
            }

            task = std::move(tasks.front());
            tasks.pop();
        } // 락 해제

        task();
        return true; // 작업 실행함
    }

private:
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    bool running;
};



/* EXAMPLE
 *
#include <thread>
#include <chrono>

TaskQueue main_thread_task_queue;

// [가정] 네트워크 스레드
// 짧은 시간에 많은 작업을 보냅니다.
void network_thread_loop() {
    for (int i = 0; i < 20; ++i) { // 20개의 작업
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 20ms마다 전송
        std::cout << "[네트워크 스레드] " << i << "번 작업 포스팅\n";

        main_thread_task_queue.post([i] {
            // 메인 스레드에서 실행될 작업
            std::cout << "[메인 스레드] " << i << "번 작업 실행!\n";
        });
    }
}

// [애플리케이션 메인 루프]
int main() {
    std::thread network_worker(network_thread_loop);

    bool app_running = true;


    while (app_running) {
        main_thread_task_queue.run_one();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    main_thread_task_queue.stop();
    network_worker.join();

    std::cout << "모든 작업 처리 완료.\n";
    return 0;
}

*/