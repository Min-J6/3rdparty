#include "lib/imgui/icon.h"
#include "lib/imgui/imgui_app.h"
#include "lib/imgui/ImCoolBar.h"
#include "node_editor.h"
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <deque>
#include <cstring>

// 크로스 플랫폼 시리얼 포트 처리
#ifdef _WIN32
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <termios.h>
    #include <unistd.h>
    #include <dirent.h>
    #include <sys/ioctl.h>
#endif

// 시리얼 포트 클래스
class SerialPort {
private:
#ifdef _WIN32
    HANDLE hSerial;
#else
    int fd;
#endif
    bool connected;
    std::string port_name;
    int baud_rate;

public:
    SerialPort() : connected(false), baud_rate(9600) {
#ifdef _WIN32
        hSerial = INVALID_HANDLE_VALUE;
#else
        fd = -1;
#endif
    }

    ~SerialPort() {
        disconnect();
    }

    bool connect(const std::string& port, int baud) {
        port_name = port;
        baud_rate = baud;

#ifdef _WIN32
        hSerial = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            return false;
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return false;
        }

        dcbSerialParams.BaudRate = baud;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return false;
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
            return false;
        }

        connected = true;
        return true;
#else
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd == -1) {
            return false;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            fd = -1;
            return false;
        }

        speed_t speed;
        switch(baud) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B9600; break;
        }

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            fd = -1;
            return false;
        }

        connected = true;
        return true;
#endif
    }

    void disconnect() {
        if (!connected) return;

#ifdef _WIN32
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
#else
        if (fd != -1) {
            close(fd);
            fd = -1;
        }
#endif
        connected = false;
    }

    bool isConnected() const {
        return connected;
    }

    int readData(char* buffer, int size) {
        if (!connected) return 0;

#ifdef _WIN32
        DWORD bytesRead = 0;
        if (!ReadFile(hSerial, buffer, size, &bytesRead, NULL)) {
            return 0;
        }
        return bytesRead;
#else
        int n = read(fd, buffer, size);
        return (n > 0) ? n : 0;
#endif
    }

    bool writeData(const char* data, int size) {
        if (!connected) return false;

#ifdef _WIN32
        DWORD bytesSent = 0;
        if (!WriteFile(hSerial, data, size, &bytesSent, NULL)) {
            return false;
        }
        return bytesSent == size;
#else
        int n = write(fd, data, size);
        return n == size;
#endif
    }

    static std::vector<std::string> getAvailablePorts() {
        std::vector<std::string> ports;

#ifdef _WIN32
        for (int i = 1; i <= 256; i++) {
            std::string port = "COM" + std::to_string(i);
            HANDLE hPort = CreateFileA(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
            if (hPort != INVALID_HANDLE_VALUE) {
                ports.push_back(port);
                CloseHandle(hPort);
            }
        }
#else
        DIR* dir = opendir("/dev");
        if (dir) {
            struct dirent* entry;
            while ((entry = readdir(dir)) != NULL) {
                std::string name = entry->d_name;
                if (name.find("ttyUSB") == 0 || name.find("ttyACM") == 0 || name.find("ttyS") == 0) {
                    ports.push_back("/dev/" + name);
                }
            }
            closedir(dir);
        }
#endif
        return ports;
    }
};

int main(int, char**) {
    // 백그라운드에서 ImGui 앱 시작
    ImguiApp::start_background("노드 에디터 데모");

    // 시리얼 포트 관련 변수
    static SerialPort serial;
    static std::vector<std::string> available_ports;
    static int selected_port_idx = 0;
    static int baud_rates[] = {9600, 19200, 38400, 57600, 115200};
    static const char* baud_rate_names[] = {"9600", "19200", "38400", "57600", "115200"};
    static int selected_baud_idx = 4; // 기본값 115200

    // 수신 데이터 버퍼
    static std::deque<std::string> received_lines;
    static std::string current_line;
    static const int MAX_LINES = 1000;

    // 송신 데이터
    static char send_buffer[256] = "";
    static bool auto_scroll = true;
    static bool show_timestamp = false;

    // 배경 크기 입력 변수
    static float bg_size[2] = {0, 0};

    // 초기 포트 스캔
    available_ports = SerialPort::getAvailablePorts();

    while (ImguiApp::is_running()) {
        // 매 루프마다 렌더링 콜백 설정
        ImguiApp::show_imgui([&](){
            ImGui::Begin("시리얼 모니터");

            // 연결 설정 섹션
            ImGui::Text("연결 설정");
            ImGui::Separator();

            // 포트 새로고침 버튼
            if (ImGui::Button(" 포트 새로고침")) {
                available_ports = SerialPort::getAvailablePorts();
                if (selected_port_idx >= available_ports.size()) {
                    selected_port_idx = 0;
                }
            }
            ImGui::SameLine();

            // 포트 선택 콤보박스
            ImGui::SetNextItemWidth(200);
            if (ImGui::BeginCombo("##port", available_ports.empty() ? "포트 없음" : available_ports[selected_port_idx].c_str())) {
                for (int i = 0; i < available_ports.size(); i++) {
                    bool is_selected = (selected_port_idx == i);
                    if (ImGui::Selectable(available_ports[i].c_str(), is_selected)) {
                        selected_port_idx = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::SameLine();

            // 보드레이트 선택
            ImGui::SetNextItemWidth(120);
            if (ImGui::BeginCombo("##baud", baud_rate_names[selected_baud_idx])) {
                for (int i = 0; i < 5; i++) {
                    bool is_selected = (selected_baud_idx == i);
                    if (ImGui::Selectable(baud_rate_names[i], is_selected)) {
                        selected_baud_idx = i;
                    }
                    if (is_selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::SameLine();

            // 연결/해제 버튼
            if (serial.isConnected()) {
                if (ImGui::Button(" 연결 해제")) {
                    serial.disconnect();
                    received_lines.push_back("[시스템] 연결 해제됨");
                }
            } else {
                if (ImGui::Button(" 연결") && !available_ports.empty()) {
                    if (serial.connect(available_ports[selected_port_idx], baud_rates[selected_baud_idx])) {
                        received_lines.push_back("[시스템] " + available_ports[selected_port_idx] + " 연결됨 (" + baud_rate_names[selected_baud_idx] + " baud)");
                        current_line.clear();
                    } else {
                        received_lines.push_back("[오류] 연결 실패");
                    }
                }
            }

            ImGui::Spacing();
            ImGui::Separator();

            // 수신 데이터 표시 영역
            ImGui::Text("수신 데이터");
            ImGui::SameLine();
            ImGui::Checkbox("자동 스크롤", &auto_scroll);
            ImGui::SameLine();
            ImGui::Checkbox("타임스탬프", &show_timestamp);
            ImGui::SameLine();
            if (ImGui::Button(" 지우기")) {
                received_lines.clear();
            }

            // 수신 데이터 읽기
            if (serial.isConnected()) {
                char buffer[1024];
                int bytes_read = serial.readData(buffer, sizeof(buffer) - 1);
                if (bytes_read > 0) {
                    buffer[bytes_read] = '\0';
                    for (int i = 0; i < bytes_read; i++) {
                        if (buffer[i] == '\n') {
                            if (!current_line.empty()) {
                                if (show_timestamp) {
                                    time_t now = time(nullptr);
                                    char timestamp[32];
                                    strftime(timestamp, sizeof(timestamp), "[%H:%M:%S] ", localtime(&now));
                                    received_lines.push_back(std::string(timestamp) + current_line);
                                } else {
                                    received_lines.push_back(current_line);
                                }
                                current_line.clear();

                                // 최대 라인 수 제한
                                if (received_lines.size() > MAX_LINES) {
                                    received_lines.pop_front();
                                }
                            }
                        } else if (buffer[i] != '\r') {
                            current_line += buffer[i];
                        }
                    }
                }
            }

            // 수신 데이터 표시 창
            ImGui::BeginChild("ReceiveArea", ImVec2(0, -60), true, ImGuiWindowFlags_HorizontalScrollbar);
            for (const auto& line : received_lines) {
                ImGui::TextUnformatted(line.c_str());
            }
            if (auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
                ImGui::SetScrollHereY(1.0f);
            }
            ImGui::EndChild();

            // 송신 데이터 입력
            ImGui::Text("송신 데이터");
            ImGui::SetNextItemWidth(-80);
            bool send_on_enter = ImGui::InputText("##send", send_buffer, sizeof(send_buffer), ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();

            if ((ImGui::Button(" 전송") || send_on_enter) && serial.isConnected()) {
                std::string data = std::string(send_buffer) + "\n";
                if (serial.writeData(data.c_str(), data.length())) {
                    received_lines.push_back("[전송] " + std::string(send_buffer));
                    send_buffer[0] = '\0';
                } else {
                    received_lines.push_back("[오류] 전송 실패");
                }
            }

            ImGui::End();
        });
    }

    // ImGui 앱 종료
    ImguiApp::stop_background();

    return 0;
}

