#include "lib/imgui/icon.h"
#include "lib/imgui/imgui_app.h"
#include "lib/imgui/ImCoolBar.h"
#include "lib/imgui/icon.h"
#include "lib/serial_port.hpp"
#include <sstream>
#include <iomanip>
#include <ctime>
#include <map>
#include <deque>
#include <cstring>
#include <fstream>

#include <boost/asio.hpp>

// 매크로 구조체
struct Macro {
    char name[64] = "";
    char data[256] = "";
    bool is_hex = false;
    int line_ending = 0;
    int hotkey = 0;
};

// 단축키 정의
struct HotkeyDef {
    const char* name;
    ImGuiKey key;
    bool needs_ctrl;
    bool needs_shift;
    bool needs_alt;
};

// 사용 가능한 단축키 목록
static std::vector<HotkeyDef> available_hotkeys = {
    {"없음", ImGuiKey_None, false, false, false},

    // 숫자 키
    {"1", ImGuiKey_1, false, false, false},
    {"2", ImGuiKey_2, false, false, false},
    {"3", ImGuiKey_3, false, false, false},
    {"4", ImGuiKey_4, false, false, false},
    {"5", ImGuiKey_5, false, false, false},
    {"6", ImGuiKey_6, false, false, false},
    {"7", ImGuiKey_7, false, false, false},
    {"8", ImGuiKey_8, false, false, false},
    {"9", ImGuiKey_9, false, false, false},
    {"0", ImGuiKey_0, false, false, false},

    // 알파벳 키
    {"A", ImGuiKey_A, false, false, false},
    {"B", ImGuiKey_B, false, false, false},
    {"C", ImGuiKey_C, false, false, false},
    {"D", ImGuiKey_D, false, false, false},
    {"E", ImGuiKey_E, false, false, false},
    {"F", ImGuiKey_F, false, false, false},
    {"G", ImGuiKey_G, false, false, false},
    {"H", ImGuiKey_H, false, false, false},
    {"I", ImGuiKey_I, false, false, false},
    {"J", ImGuiKey_J, false, false, false},
    {"K", ImGuiKey_K, false, false, false},
    {"L", ImGuiKey_L, false, false, false},
    {"M", ImGuiKey_M, false, false, false},
    {"N", ImGuiKey_N, false, false, false},
    {"O", ImGuiKey_O, false, false, false},
    {"P", ImGuiKey_P, false, false, false},
    {"Q", ImGuiKey_Q, false, false, false},
    {"R", ImGuiKey_R, false, false, false},
    {"S", ImGuiKey_S, false, false, false},
    {"T", ImGuiKey_T, false, false, false},
    {"U", ImGuiKey_U, false, false, false},
    {"V", ImGuiKey_V, false, false, false},
    {"W", ImGuiKey_W, false, false, false},
    {"X", ImGuiKey_X, false, false, false},
    {"Y", ImGuiKey_Y, false, false, false},
    {"Z", ImGuiKey_Z, false, false, false},

    // 펑션 키
    {"F1", ImGuiKey_F1, false, false, false},
    {"F2", ImGuiKey_F2, false, false, false},
    {"F3", ImGuiKey_F3, false, false, false},
    {"F4", ImGuiKey_F4, false, false, false},
    {"F5", ImGuiKey_F5, false, false, false},
    {"F6", ImGuiKey_F6, false, false, false},
    {"F7", ImGuiKey_F7, false, false, false},
    {"F8", ImGuiKey_F8, false, false, false},
    {"F9", ImGuiKey_F9, false, false, false},
    {"F10", ImGuiKey_F10, false, false, false},
    {"F11", ImGuiKey_F11, false, false, false},
    {"F12", ImGuiKey_F12, false, false, false},

    // Ctrl + 숫자
    {"Ctrl+1", ImGuiKey_1, true, false, false},
    {"Ctrl+2", ImGuiKey_2, true, false, false},
    {"Ctrl+3", ImGuiKey_3, true, false, false},
    {"Ctrl+4", ImGuiKey_4, true, false, false},
    {"Ctrl+5", ImGuiKey_5, true, false, false},
    {"Ctrl+6", ImGuiKey_6, true, false, false},
    {"Ctrl+7", ImGuiKey_7, true, false, false},
    {"Ctrl+8", ImGuiKey_8, true, false, false},
    {"Ctrl+9", ImGuiKey_9, true, false, false},
    {"Ctrl+0", ImGuiKey_0, true, false, false},

    // Ctrl + 알파벳
    {"Ctrl+Q", ImGuiKey_Q, true, false, false},
    {"Ctrl+W", ImGuiKey_W, true, false, false},
    {"Ctrl+E", ImGuiKey_E, true, false, false},
    {"Ctrl+R", ImGuiKey_R, true, false, false},
    {"Ctrl+T", ImGuiKey_T, true, false, false},
    {"Ctrl+Y", ImGuiKey_Y, true, false, false},
    {"Ctrl+U", ImGuiKey_U, true, false, false},
    {"Ctrl+I", ImGuiKey_I, true, false, false},
    {"Ctrl+O", ImGuiKey_O, true, false, false},
    {"Ctrl+P", ImGuiKey_P, true, false, false},
    {"Ctrl+D", ImGuiKey_D, true, false, false},
    {"Ctrl+F", ImGuiKey_F, true, false, false},
    {"Ctrl+G", ImGuiKey_G, true, false, false},
    {"Ctrl+H", ImGuiKey_H, true, false, false},
    {"Ctrl+J", ImGuiKey_J, true, false, false},
    {"Ctrl+K", ImGuiKey_K, true, false, false},
    {"Ctrl+L", ImGuiKey_L, true, false, false},
    {"Ctrl+B", ImGuiKey_B, true, false, false},
    {"Ctrl+N", ImGuiKey_N, true, false, false},
    {"Ctrl+M", ImGuiKey_M, true, false, false},

    // Ctrl+Shift + 숫자
    {"Ctrl+Shift+1", ImGuiKey_1, true, true, false},
    {"Ctrl+Shift+2", ImGuiKey_2, true, true, false},
    {"Ctrl+Shift+3", ImGuiKey_3, true, true, false},
    {"Ctrl+Shift+4", ImGuiKey_4, true, true, false},
    {"Ctrl+Shift+5", ImGuiKey_5, true, true, false},

    // Alt + 숫자
    {"Alt+1", ImGuiKey_1, false, false, true},
    {"Alt+2", ImGuiKey_2, false, false, true},
    {"Alt+3", ImGuiKey_3, false, false, true},
    {"Alt+4", ImGuiKey_4, false, false, true},
    {"Alt+5", ImGuiKey_5, false, false, true},
    {"Alt+6", ImGuiKey_6, false, false, true},
    {"Alt+7", ImGuiKey_7, false, false, true},
    {"Alt+8", ImGuiKey_8, false, false, true},
    {"Alt+9", ImGuiKey_9, false, false, true},
    {"Alt+0", ImGuiKey_0, false, false, true},

    // 특수 키
    {"Insert", ImGuiKey_Insert, false, false, false},
    {"Delete", ImGuiKey_Delete, false, false, false},
    {"Home", ImGuiKey_Home, false, false, false},
    {"End", ImGuiKey_End, false, false, false},
    {"PageUp", ImGuiKey_PageUp, false, false, false},
    {"PageDown", ImGuiKey_PageDown, false, false, false},

    // 방향키
    {"Up", ImGuiKey_UpArrow, false, false, false},
    {"Down", ImGuiKey_DownArrow, false, false, false},
    {"Left", ImGuiKey_LeftArrow, false, false, false},
    {"Right", ImGuiKey_RightArrow, false, false, false},

    // 기타 키
    {"Space", ImGuiKey_Space, false, false, false},
    {"Backspace", ImGuiKey_Backspace, false, false, false},
    {"Tab", ImGuiKey_Tab, false, false, false},
};


// 헬퍼 함수: HEX 문자열을 바이트 배열로 변환
std::vector<char> hexString_to_bytes(const std::string& hex) {
    std::vector<char> bytes;
    std::string hex_clean;

    for (char c : hex) {
        if (!isspace(c)) {
            hex_clean += c;
        }
    }

    for (size_t i = 0; i < hex_clean.length(); i += 2) {
        if (i + 1 < hex_clean.length()) {
            std::string byte_str = hex_clean.substr(i, 2);
            char byte = (char)strtol(byte_str.c_str(), nullptr, 16);
            bytes.push_back(byte);
        }
    }
    return bytes;
}

// 헬퍼 함수: 바이트 배열을 HEX 문자열로 변환
std::string bytes_to_hexString(const std::vector<char>& bytes) {
    std::stringstream ss;
    for (unsigned char byte : bytes) {
        ss << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
    }
    return ss.str();
}

// 헬퍼 함수: 현재 시간 문자열 얻기
std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}






class SerialMonitor {
public:
    SerialMonitor(boost::asio::io_context& io) : serial(io), heartbeat_timer_(io) {
        available_ports = SerialPort::getAvailablePorts();

        load_macros_from_yaml();

        serial.on_receive = [this](const std::vector<char>& data)
        {
            add_received_buffer(std::string(data.begin(), data.end()));
        };

        serial.on_error = [this](const std::string& error_msg)
        {
            add_received_buffer("[시스템] 에러: " + error_msg);
        };

        serial.on_connect = [this]()
        {
            add_received_buffer("[시스템] 연결됨: " + available_ports[selected_port_idx] + " (" + baud_rate_names[selected_baud_idx] + " baud)");
        };

        serial.on_disconnect = [this]()
        {
            if (heartbeat_enabled)
            {
                heartbeat_enabled = false;
                heartbeat_timer_.cancel();
            }

            add_received_buffer("[시스템] 연결 해제됨");
            add_received_buffer("---------------------------------");
        };
    }

    ~SerialMonitor() {
        save_macros_to_yaml();

        if (serial.is_connected())
            serial.disconnect();
    }

    void render() {
        // 매크로 단축키 처리 (키 감지 모드가 아닐 때만)
        if (detecting_macro_idx == -1) {
            for (int i = 0; i < macros.size(); i++) {
                if (macros[i].hotkey > 0 && macros[i].hotkey < available_hotkeys.size()) {
                    const auto& hotkey = available_hotkeys[macros[i].hotkey];
                    if (is_hotkey_pressed(hotkey)) {
                        send_data(macros[i].data, macros[i].is_hex, macros[i].line_ending);
                    }
                }
            }
        }

        // 키 감지 모드일 때 키 입력 감지
        if (detecting_macro_idx != -1) {
            int pressed_key = detect_pressed_key();
            if (pressed_key > 0) {
                macros[detecting_macro_idx].hotkey = pressed_key;
                detecting_macro_idx = -1;
            }
        }

        // 매크로 설정 창
        {
            ImGui::Begin("매크로 설정");

            ImGui::Text("매크로를 설정하면 단축키로 언제든 빠르게 보낼 수 있어요.");

            ImGui::Dummy(ImVec2(0, 1));


            // 상단 버튼들 (저장, 불러오기, 리셋)
            if (ImGui::Button(ICON_MD_SAVE " 저장")) {
                save_macros_to_yaml();
            }
            ImGui::SameLine();
            if (ImGui::Button(ICON_MD_FOLDER_OPEN " 불러오기")) {
                load_macros_from_yaml();
            }
            ImGui::SameLine();
            if (ImGui::Button(ICON_MD_REFRESH " 리셋")) {
                reset_macros();
            }

            ImGui::Dummy(ImVec2(0, 10));



            // 매크로 목록
            for (int i = 0; i < macros.size(); i++) {
                ImGui::PushID(i);

                ImGui::Text("매크로 %d", i + 1);
                ImGui::Separator();

                ImGui::SetNextItemWidth(150);
                ImGui::Text("이름");
                ImGui::SameLine(50);
                ImGui::InputText("##이름", macros[i].name, sizeof(macros[i].name));
                ImGui::SameLine();

                ImGui::Checkbox("HEX", &macros[i].is_hex);

                ImGui::SetNextItemWidth(300);
                ImGui::Text("데이터");
                ImGui::SameLine(50);
                ImGui::InputText("##데이터", macros[i].data, sizeof(macros[i].data));
                ImGui::SameLine();

                ImGui::SetNextItemWidth(75);
                ImGui::Combo("##라인 엔딩", &macros[i].line_ending, line_ending_names, 4);

                // 중복 확인
                bool has_duplicate = false;
                int dup_idx = -1;
                if (macros[i].hotkey > 0) {
                    for (int j = 0; j < macros.size(); j++) {
                        if (i != j && macros[j].hotkey == macros[i].hotkey) {
                            has_duplicate = true;
                            dup_idx = j;
                            break;
                        }
                    }
                }

                if (has_duplicate) {
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.5f, 0.0f, 1.0f));
                    ImGui::Text("단축키: %s (매크로 %d와 중복)",
                               available_hotkeys[macros[i].hotkey].name, dup_idx + 1);
                    ImGui::PopStyleColor();
                } else {
                    ImGui::Text("단축키: %s", available_hotkeys[macros[i].hotkey].name);
                }
                ImGui::SameLine();

                if (detecting_macro_idx == i) {
                    if (ImGui::Button( " 취소 ")) {
                        detecting_macro_idx = -1;
                    }
                } else {
                    if (ImGui::Button( " 키 등록 ")) {
                        detecting_macro_idx = i;
                    }
                }

                ImGui::SameLine();
                if (ImGui::Button( " 해제 ")) {
                    macros[i].hotkey = 0;
                }

                ImGui::SameLine();
                if (ImGui::Button( " 보내기 ")) {
                    send_data(macros[i].data, macros[i].is_hex, macros[i].line_ending);
                }

                ImGui::Spacing();
                ImGui::PopID();

                ImGui::Dummy(ImVec2(0, 10));
            }

            ImGui::End();
        }


        // 시리얼 모니터 창
        {
            ImGui::Begin("시리얼 모니터", NULL);

            ImGui::Text("연결 설정");

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

            // 시리얼 포트 리스트 갱신 버튼
            if (ImGui::Button(ICON_MD_SYNC)) {
                available_ports = SerialPort::getAvailablePorts();
                if (selected_port_idx >= available_ports.size()) {
                    selected_port_idx = 0;
                }
            }

            ImGui::SameLine();

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

            // 포트 연결/해제 버튼
            if (serial.is_connected()) {
                if (ImGui::Button(ICON_MD_LINK_OFF " 연결 해제")) {
                    serial.disconnect();

                    // 하트비트 중지
                    if (heartbeat_enabled) {
                        heartbeat_enabled = false;
                        heartbeat_timer_.cancel();
                    }
                }
            } else {
                if (ImGui::Button(ICON_MD_LINK " 포트 연결 ") && !available_ports.empty()) {
                    serial.connect(available_ports[selected_port_idx], baud_rates[selected_baud_idx]);
                }
            }

            ImGui::Spacing();
            ImGui::Separator();

            ImGui::Dummy(ImVec2(0, 10));

            ImGui::Text("하트비트 설정");

            // 하트비트 타이머 설정
            if (heartbeat_enabled) {
                if (ImGui::Button(ICON_MD_ECG_HEART " 하트비트 중지")) {
                    heartbeat_enabled = false;
                    heartbeat_timer_.cancel();

                    // 버퍼 업데이트
                    add_received_buffer("[시스템] 하트비트 중지됨");
                }
            }
            else {
                if (ImGui::Button(ICON_MD_ECG_HEART " 하트비트 시작") && serial.is_connected()) {
                    heartbeat_enabled = true;

                    // 버퍼 업데이트
                    add_received_buffer("[시스템] 하트비트 시작됨 (간격: " + std::to_string(heartbeat_interval) + "ms)");

                    // 하트비트 타이머 시작
                    start_heartbeat_timer();
                }
            }

            ImGui::SameLine();
            ImGui::Checkbox("HEX##hb", &heartbeat_as_hex);

            ImGui::SetNextItemWidth(200);
            ImGui::InputText("##hb_data", heartbeat_buffer, sizeof(heartbeat_buffer));
            ImGui::SameLine();

            ImGui::SetNextItemWidth(58);
            ImGui::DragInt("간격(ms)##hb", &heartbeat_interval, 100, 100, 5000);

            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::Combo("##hb_ending", &heartbeat_line_ending, line_ending_names, 4);


            ImGui::Spacing();
            ImGui::Separator();

            ImGui::Dummy(ImVec2(0, 10));

            ImGui::Text("수신 데이터");
            ImGui::SameLine();
            if (ImGui::Button(ICON_MD_DELETE " 지우기")) {
                received_lines.clear();
                received_text_buffer[0] = '\0';
            }

            ImGui::SameLine(0, 15);

            ImGui::Checkbox("HEX##recv", &receive_as_hex);

            ImGui::SameLine(0, 15);

            ImGui::Checkbox("자동 스크롤", &auto_scroll);

            ImGui::SameLine(0, 15);

            ImGui::Checkbox("타임 스탬프", &show_timestamp);



            // 송수신 데이터 표시
            if (ImGui::BeginChild("ReceiveArea", ImVec2(0, -80), true, ImGuiWindowFlags_HorizontalScrollbar)) {
                for (const auto& line : received_lines) {
                    ImGui::TextUnformatted(line.c_str());
                }

                // 자동 스크롤 처리
                if (scroll_to_bottom || (auto_scroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()))
                    ImGui::SetScrollHereY(1.0f);
                scroll_to_bottom = false;
            }
            ImGui::EndChild();

            ImGui::Dummy(ImVec2(0, 10));

            ImGui::Text("송신 데이터");
            ImGui::SameLine();
            ImGui::Checkbox("HEX##send", &send_as_hex);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120);
            ImGui::Combo("##send_ending", &send_line_ending, line_ending_names, 4);


            // 데이터 표시 창
            ImGui::SetNextItemWidth(-70);
            if (ImGui::InputText("##send", send_buffer, sizeof(send_buffer), ImGuiInputTextFlags_EnterReturnsTrue)) {
                send_data(send_buffer, send_as_hex, send_line_ending);
                send_buffer[0] = '\0';
                ImGui::SetKeyboardFocusHere(-1);
            }
            ImGui::SameLine();


            // 송신 버튼
            if (ImGui::Button(ICON_MD_SEND "보내기") && serial.is_connected()) {
                send_data(send_buffer, send_as_hex, send_line_ending);
            }

            ImGui::End();
        }
    }

private:
    // 시리얼 포트로 데이터 전송
    void send_data(const std::string& data_str, bool is_hex, int line_ending) {
        if (!serial.is_connected()) return;


        std::vector<char> data;


        if (is_hex) {
            data = hexString_to_bytes(data_str);
        } else {
            data = std::vector<char>(data_str.begin(), data_str.end());
        }


        if (line_ending == 1) data.push_back('\n');
        else if (line_ending == 2) data.push_back('\r');
        else if (line_ending == 3) { data.push_back('\r'); data.push_back('\n'); }


        serial.send(data);


        if (data.size() > 0)
            add_received_buffer(data_str);
    }

    // Receive buffer 업데이트
    void add_received_buffer(const std::string& line_str) {
        received_lines.push_back(line_str);
        size_t buffer_size = sizeof(received_text_buffer);


        std::string temp;
        for (const auto& l : received_lines) {
            temp += l + "\n";
        }


        strncpy(received_text_buffer, temp.c_str(), buffer_size - 1);
        received_text_buffer[buffer_size - 1] = '\0';
    }

    // 눌린 단축키 확인 (키 감지 모드)
    int detect_pressed_key() {
        ImGuiIO& io = ImGui::GetIO();

        // 모든 단축키를 순회하며 눌린 키 찾기
        for (int i = 1; i < available_hotkeys.size(); i++) {
            const auto& hotkey = available_hotkeys[i];

            if (hotkey.key == ImGuiKey_None) continue;

            bool ctrl_match = hotkey.needs_ctrl ? io.KeyCtrl : !io.KeyCtrl;
            bool shift_match = hotkey.needs_shift ? io.KeyShift : !io.KeyShift;
            bool alt_match = hotkey.needs_alt ? io.KeyAlt : !io.KeyAlt;

            if (ImGui::IsKeyPressed(hotkey.key) && ctrl_match && shift_match && alt_match) {
                return i;
            }
        }

        return 0;
    }

    // 단축키가 눌렸는지 확인
    bool is_hotkey_pressed(const HotkeyDef& hotkey) {
        if (hotkey.key == ImGuiKey_None) return false;

        ImGuiIO& io = ImGui::GetIO();

        bool ctrl_match = hotkey.needs_ctrl ? io.KeyCtrl : !io.KeyCtrl;
        bool shift_match = hotkey.needs_shift ? io.KeyShift : !io.KeyShift;
        bool alt_match = hotkey.needs_alt ? io.KeyAlt : !io.KeyAlt;

        if (io.WantTextInput) return false;

        return ImGui::IsKeyPressed(hotkey.key) && ctrl_match && shift_match && alt_match;
    }

    // 하트비트 타이머 시작
    void start_heartbeat_timer() {
        if (!heartbeat_enabled || !serial.is_connected()) return;

        heartbeat_timer_.expires_after(std::chrono::milliseconds(heartbeat_interval));
        heartbeat_timer_.async_wait([this](const boost::system::error_code& ec) {
            if (!ec && heartbeat_enabled && serial.is_connected()) {
                // 하트비트 데이터 전송
                std::vector<char> hb_data;

                if (heartbeat_as_hex) {
                    hb_data = hexString_to_bytes(std::string(heartbeat_buffer));
                } else {
                    hb_data = std::vector<char>(heartbeat_buffer, heartbeat_buffer + strlen(heartbeat_buffer));
                }

                if (heartbeat_line_ending == 1) hb_data.push_back('\n');
                else if (heartbeat_line_ending == 2) hb_data.push_back('\r');
                else if (heartbeat_line_ending == 3) { hb_data.push_back('\r'); hb_data.push_back('\n'); }


                serial.send(hb_data);

                // 다음 하트비트 예약
                start_heartbeat_timer();
            }
        });
    }

    // 매크로 설정 파일 불러오기
    void load_macros_from_yaml() {
         std::ifstream file(macro_config_file);
        if (!file.is_open()) return;

        std::string line;
        int current_index = -1;
        Macro temp_macro;

        while (std::getline(file, line)) {
            // 공백 제거
            size_t start = line.find_first_not_of(" \t");
            if (start == std::string::npos) continue;
            line = line.substr(start);

            if (line.find("- index:") == 0) {
                // 새 매크로 시작
                if (current_index >= 0 && current_index < macros.size()) {
                    macros[current_index] = temp_macro;
                }

                // index 파싱
                size_t pos = line.find(":");
                if (pos != std::string::npos) {
                    current_index = std::stoi(line.substr(pos + 1));
                    temp_macro = Macro(); // 초기화
                }
            }
            else if (line.find("name:") == 0) {
                size_t pos = line.find(":");
                if (pos != std::string::npos) {
                    std::string value = line.substr(pos + 1);
                    // 앞뒤 공백 제거
                    size_t first = value.find_first_not_of(" \t\"");
                    size_t last = value.find_last_not_of(" \t\"");
                    if (first != std::string::npos && last != std::string::npos) {
                        value = value.substr(first, last - first + 1);
                        strncpy(temp_macro.name, value.c_str(), sizeof(temp_macro.name) - 1);
                    }
                }
            }
            else if (line.find("data:") == 0) {
                size_t pos = line.find(":");
                if (pos != std::string::npos) {
                    std::string value = line.substr(pos + 1);
                    size_t first = value.find_first_not_of(" \t\"");
                    size_t last = value.find_last_not_of(" \t\"");
                    if (first != std::string::npos && last != std::string::npos) {
                        value = value.substr(first, last - first + 1);
                        strncpy(temp_macro.data, value.c_str(), sizeof(temp_macro.data) - 1);
                    }
                }
            }
            else if (line.find("is_hex:") == 0) {
                size_t pos = line.find(":");
                if (pos != std::string::npos) {
                    std::string value = line.substr(pos + 1);
                    temp_macro.is_hex = (value.find("true") != std::string::npos);
                }
            }
            else if (line.find("line_ending:") == 0) {
                size_t pos = line.find(":");
                if (pos != std::string::npos) {
                    temp_macro.line_ending = std::stoi(line.substr(pos + 1));
                }
            }
            else if (line.find("hotkey:") == 0) {
                size_t pos = line.find(":");
                if (pos != std::string::npos) {
                    temp_macro.hotkey = std::stoi(line.substr(pos + 1));
                }
            }
        }

        // 마지막 매크로 저장
        if (current_index >= 0 && current_index < macros.size()) {
            macros[current_index] = temp_macro;
        }

        file.close();
    }

    // 매크로 설정 파일 저장
    void save_macros_to_yaml() {
        std::ofstream file(macro_config_file);
        if (!file.is_open()) return;

        file << "macros:\n";
        for (int i = 0; i < macros.size(); i++) {
            file << "  - index: " << i << "\n";
            file << "    name: \"" << macros[i].name << "\"\n";
            file << "    data: \"" << macros[i].data << "\"\n";
            file << "    is_hex: " << (macros[i].is_hex ? "true" : "false") << "\n";
            file << "    line_ending: " << macros[i].line_ending << "\n";
            file << "    hotkey: " << macros[i].hotkey << "\n";
        }

        file.close();
    }

    // 매크로 리셋
    void reset_macros() {
        macros.clear();
        macros.resize(10);
    }


    std::vector<std::string> available_ports;
    int selected_port_idx = 0;
    int baud_rates[5] = {9600, 19200, 38400, 57600, 115200};
    const char* baud_rate_names[5] = {"9600", "19200", "38400", "57600", "115200"};
    int selected_baud_idx = 4;

    std::deque<std::string> received_lines;
    const int MAX_LINES = 1000;

    // 수신 데이터를 문자열로 저장하기 위한 버퍼 (최대 100KB)
    char received_text_buffer[1024 * 100] = "";


    // 송신 데이터 관련
    char send_buffer[256] = "";
    bool send_as_hex = false;
    int send_line_ending = 0;
    const char* line_ending_names[4] = {"없음", "LF", "CR", "CR+LF"};


    // 수신 데이터 관련
    bool receive_as_hex = false;
    bool auto_scroll = true;
    bool scroll_to_bottom = false;
    bool show_timestamp = false;


    // 하트 비트 관련
    char heartbeat_buffer[256] = "PING";
    bool heartbeat_as_hex = false;
    int heartbeat_interval = 1000;
    int heartbeat_line_ending = 0;
    bool heartbeat_enabled = false;
    boost::asio::steady_timer heartbeat_timer_;


    // 매크로 관련
    std::vector<Macro> macros = std::vector<Macro>(10);
    int detecting_macro_idx = -1; // 키 감지 중인 매크로 인덱스
    int duplicate_warning_macro = -1; // 중복 경고 표시할 매크로
    int duplicate_with_macro = -1; // 중복된 다른 매크로
    const std::string macro_config_file = "macros.yaml";


    SerialPort serial;
};






int main(int, char**) {
    ImguiApp::start_background("시리얼 모니터", ImVec2(480, 720));

    boost::asio::io_context io_context;
    SerialMonitor sm(io_context);

    // 1. work_guard를 io_thread 생성 전에 만듭니다.
    auto work_guard = boost::asio::make_work_guard(io_context);

    // 2. io_context.run()을 실행할 스레드를 시작합니다.
    std::thread io_thread([&io_context]() {
        io_context.run();
    });

    // GUI 메인 루프
    while (ImguiApp::is_running()) {
        ImguiApp::show_imgui([&]() {
            sm.render();
        });
    }

    ImguiApp::stop_background();

    // 3. GUI가 종료되면 work_guard를 리셋하여 io_context.run()이 반환되도록 합니다.
    work_guard.reset();

    // 4. io_context를 정지시키고 스레드가 완전히 종료될 때까지 기다립니다.
    io_context.stop();
    if (io_thread.joinable()) {
        io_thread.join();
    }

    return 0;
}


