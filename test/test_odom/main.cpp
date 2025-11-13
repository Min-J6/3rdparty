#include <iostream>
#include <cmath>
#include <thread>      // 스레드
#include <mutex>       // 뮤텍스 (데이터 보호)
#include <chrono>      // 시간 (sleep, dt)
#include <atomic>      // 원자적 연산 (스레드 종료 플래그)
#include <iomanip>     // 출력 포맷팅 (setprecision)

// --- M_PI 정의 (이전과 동일) ---
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// --- RobotParams, Pose 구조체 (이전과 동일) ---
struct RobotParams {
    double wheel_base = 1.0;
    double front_track_width = 0.6;
    double rear_track_width = 0.6;
    double wheel_diameter = 0.2;
};


struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0; // 라디안(Radian)
};



double normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}


void updateOdomFromLinearSpeed(Pose& pose,
                               double v_fl, double v_fr, double v_rl, double v_rr,
                               double delta_fl, double delta_fr, double delta_rl, double delta_rr,
                               double dt,
                               const RobotParams& params)
{
    // 각 바퀴의 x속도와 y 속도를 분리
    double vx_fl = v_fl * std::cos(delta_fl);
    double vy_fl = v_fl * std::sin(delta_fl);
    double vx_fr = v_fr * std::cos(delta_fr);
    double vy_fr = v_fr * std::sin(delta_fr);
    double vx_rl = v_rl * std::cos(delta_rl);
    double vy_rl = v_rl * std::sin(delta_rl);
    double vx_rr = v_rr * std::cos(delta_rr);
    double vy_rr = v_rr * std::sin(delta_rr);


    // 로봇의 x속도와 y속도를 업데이트 (평균 방식)
    double vx_robot = (vx_fl + vx_fr + vx_rl + vx_rr) / 4.0;
    double vy_robot = (vy_fl + vy_fr + vy_rl + vy_rr) / 4.0;


    // 제자리 회전 방정식을 이용하여 로봇의 속도를 업데이트 (wz = wz_robot)
    double wz_from_x_front = (vx_fr - vx_fl) / params.front_track_width;
    double wz_from_x_rear = (vx_rr - vx_rl) / params.rear_track_width;
    double vy_front_avg = (vy_fl + vy_fr) / 2.0;
    double vy_rear_avg  = (vy_rl + vy_rr) / 2.0;
    double wz_from_y = (vy_front_avg - vy_rear_avg) / params.wheel_base;
    double wz_robot = (wz_from_x_front + wz_from_x_rear + wz_from_y) / 3.0;


    // 로봇의 위치와 각도를 업데이트 (dt는 시간 간격)
    double mid_theta = pose.theta + (wz_robot * dt / 2.0);
    double cos_mid_theta = std::cos(mid_theta);
    double sin_mid_theta = std::sin(mid_theta);

    double dx_odom = (vx_robot * cos_mid_theta - vy_robot * sin_mid_theta) * dt;
    double dy_odom = (vx_robot * sin_mid_theta + vy_robot * cos_mid_theta) * dt;
    double dtheta_odom = wz_robot * dt;


    // 최종 로봇의 상대적 위치와 각도 업데이트
    pose.x += dx_odom;
    pose.y += dy_odom;
    pose.theta += dtheta_odom;
    pose.theta = normalizeAngle(pose.theta);
}



// 1. 스레드 간에 공유될 데이터
Pose g_robotPose;               // 로봇의 현재 상태 (전역 변수)
std::mutex g_poseMutex;         // g_robotPose를 보호하기 위한 뮤텍스
std::atomic<bool> g_keepRunning(true); // 시뮬레이션 스레드 종료 플래그

/**
 * @brief 시뮬레이션 스레드에서 실행될 메인 루프
 * 10Hz (100ms) 주기로 가상의 입력을 생성하고 g_robotPose를 업데이트합니다.
 */
void simulationLoop(const RobotParams& params) {

    // 10Hz = 100ms 주기
    const auto loop_duration = std::chrono::milliseconds(100);
    const double dt = std::chrono::duration<double>(loop_duration).count(); // dt = 0.1

    double simulation_time = 0.0;

    std::cout << "[Sim Thread] 시뮬레이션 스레드 시작 (10Hz)" << std::endl;

    while (g_keepRunning) {
        auto loop_start = std::chrono::steady_clock::now();

        // 시뮬레이션 시나리오 (시간에 따라 다른 움직임)
        double v_fl=0, v_fr=0, v_rl=0, v_rr=0;
        double d_fl=0, d_fr=0, d_rl=0, d_rr=0;
        double angle_90_deg = M_PI / 2.0;

        if (simulation_time < 5.0) {
            // 0~5초: 1.0 m/s로 직진
            v_fl = 1.0; v_fr = 1.0; v_rl = 1.0; v_rr = 1.0;
            d_fl = 0.0; d_fr = 0.0; d_rl = 0.0; d_rr = 0.0;
        } else if (simulation_time < 8.0) {
            // 5~8초: 제자리 회전 (시계 방향)
            // wz ≈ (v_R - v_L) / Track = (-0.5 - 0.5) / 0.6 = -1.66 rad/s
            v_fl = 0.5; v_fr = -0.5; v_rl = 0.5; v_rr = -0.5;
            d_fl = 0.0; d_fr = 0.0; d_rl = 0.0; d_rr = 0.0;
        } else if (simulation_time < 13.0) {
            // 8~13초: 게걸음 (왼쪽, 0.5 m/s)
            v_fl = 0.5; v_fr = 0.5; v_rl = 0.5; v_rr = 0.5;
            d_fl = angle_90_deg; d_fr = angle_90_deg;
            d_rl = angle_90_deg; d_rr = angle_90_deg;
        } else {
            // 13초 후: 시뮬레이션 종료
            g_keepRunning = false;
        }

        // --- 크리티컬 섹션 (Critical Section) ---
        // g_robotPose를 업데이트하기 전에 뮤텍스를 잠급니다.
        // std::lock_guard는 이 블록( { } )을 벗어날 때 자동으로 잠금을 해제합니다.
        {
            std::lock_guard<std::mutex> lock(g_poseMutex);
            updateOdomFromLinearSpeed(g_robotPose,
                                      v_fl, v_fr, v_rl, v_rr,
                                      d_fl, d_fr, d_rl, d_rr,
                                      dt, params);
        } // <- 여기서 뮤텍스 자동 잠금 해제

        simulation_time += dt;

        // 루프 주기 맞추기
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed = loop_end - loop_start;
        if (elapsed < loop_duration) {
            std::this_thread::sleep_for(loop_duration - elapsed);
        }
    }

    std::cout << "[Sim Thread] 시뮬레이션 스레드 종료." << std::endl;
}

/**
 * @brief 메인 함수
 * 시뮬레이션 스레드를 시작하고,
 * 메인 스레드에서는 0.5초마다 현재 Pose를 읽어와 출력합니다.
 */
int main() {
    RobotParams myRobotParams; // 기본값 사용

    std::cout << "[Main Thread] 시뮬레이션 스레드를 시작합니다..." << std::endl;

    // 1. 시뮬레이션 스레드 시작
    // simulationLoop 함수를 myRobotParams 파라미터와 함께 실행
    std::thread simThread(simulationLoop, myRobotParams);

    // 2. 메인 스레드 루프 (출력 담당)
    // 500ms (0.5초) 주기로 현재 Pose 값을 읽어와 출력
    while (g_keepRunning) {

        Pose currentPose; // 로컬 복사본

        // --- 크리티컬 섹션 (Critical Section) ---
        // g_robotPose를 읽어오기 전에 뮤텍스를 잠급니다.
        {
            std::lock_guard<std::mutex> lock(g_poseMutex);
            currentPose = g_robotPose; // 전역 변수를 로컬 변수로 복사
        } // <- 여기서 뮤텍스 자동 잠금 해제

        // 포맷팅하여 출력
        std::cout << "[Main Thread] Current Pose: "
                  << "x=" << std::fixed << std::setprecision(3) << currentPose.x << " m, "
                  << "y=" << std::fixed << std::setprecision(3) << currentPose.y << " m, "
                  << "theta=" << std::fixed << std::setprecision(3)
                  << currentPose.theta * (180.0 / M_PI) << " deg" // (각도(deg)로 변환하여 출력)
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 3. 시뮬레이션 스레드가 종료될 때까지 대기
    simThread.join();

    std::cout << "[Main Thread] 메인 스레드 종료." << std::endl;

    return 0;
}