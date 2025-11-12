#include "TaskQueue.hpp"

int main() {
    TaskQueue queue;

    // 조건으로 사용할 Atomic boolean
    std::atomic<bool> condition_met = false;

    // 1. 일반 작업 포스팅
    queue.post([] {
        std::cout << "[TASK 1] 저는 조건 없이 바로 실행됩니다.\n";
    });

    // 2. 조건부 작업 포스팅
    // condition_met이 true가 될 때까지 실행되지 않고 "pass"됩니다.
    queue.post_conditional(
        // 조건 함수 (bool 반환)
        [&condition_met] { return condition_met.load(); },
        // 실제 작업 (void 반환)
        [] { std::cout << "[TASK 2] (조건 충족!) 드디어 실행됩니다!\n"; }
    );

    // 3. 또 다른 일반 작업
    queue.post([] {
        std::cout << "[TASK 3] 저는 조건부 작업보다 먼저 실행될 수 있습니다.\n";
    });

    // ------------------------------------
    // 다른 스레드에서 2초 뒤에 조건을 충족시킴
    std::thread condition_setter([&condition_met] {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cout << "\n... (다른 스레드) 이제 조건을 true로 설정합니다 ...\n\n";
        condition_met.store(true);
    });
    // ------------------------------------

    // 메인 스레드에서 run_one()을 계속 호출 (루프)
    int pass_count = 0;
    while (queue.size() > 0) {
        if (!queue.run_one()) {
            // run_one()이 false를 반환 = 작업은 있지만 조건이 안 맞아서 "pass"
            pass_count++;
            // CPU 과부하 방지를 위해 잠시 대기
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::cout << "\n조건부 작업이 'pass'된 횟수: " << pass_count << "\n";

    condition_setter.join();
    return 0;
}