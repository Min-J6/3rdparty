#pragma once
#include "lib/imgui/imgui.h"
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>

// 이미지 로드 헬퍼 함수 (STB 이미지 로더 사용)
#define STB_IMAGE_IMPLEMENTATION
#include "imgui_impl_opengl3_loader.h"
#include "lib/stb/stb_image.h"

namespace NodeEditor {

// 포트 타입
enum class PortType {
    Input,
    Output
};

// 포트 구조체
struct Port {
    int id;
    int node_id;
    PortType type;
    std::string name;
    ImVec2 position;  // 캔버스 좌표

    Port(int _id, int _node_id, PortType _type, const std::string& _name)
        : id(_id), node_id(_node_id), type(_type), name(_name), position(ImVec2(0, 0)) {}
};

// 연결 구조체
struct Connection {
    int id;
    int output_port_id;
    int input_port_id;

    Connection(int _id, int _output_id, int _input_id)
        : id(_id), output_port_id(_output_id), input_port_id(_input_id) {}
};

    bool LoadTextureFromFile(const char* filename, ImTextureID* out_texture, int* out_width, int* out_height) {
        // 이미지 파일 로드
        int image_width = 0;
        int image_height = 0;
        unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
        if (image_data == NULL)
            return false;

        // OpenGL 텍스처 생성
        GLuint image_texture;
        glGenTextures(1, &image_texture);
        glBindTexture(GL_TEXTURE_2D, image_texture);

        // 텍스처 파라미터 설정
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // 텍스처 데이터 업로드
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
        stbi_image_free(image_data);

        *out_texture = (ImTextureID)(intptr_t)image_texture;
        *out_width = image_width;
        *out_height = image_height;

        return true;
    }



// 노드 구조체
struct Node {
    int id;
    ImVec2 position;
    ImVec2 size;
    std::string title;
    ImU32 color;
    bool selected;
    std::vector<int> input_ports;   // 입력 포트 ID 목록
    std::vector<int> output_ports;  // 출력 포트 ID 목록

    Node(int _id, ImVec2 _pos, const std::string& _title)
        : id(_id), position(_pos), size(ImVec2(120, 80)),
          title(_title), color(IM_COL32(100, 100, 150, 255)), selected(false) {}
};

// 노드 에디터 클래스
class Editor {
private:
    std::vector<std::shared_ptr<Node>> nodes;
    int next_node_id;

    // 포트 및 연결
    std::vector<std::shared_ptr<Port>> ports;
    std::vector<std::shared_ptr<Connection>> connections;
    int next_port_id;
    int next_connection_id;

    // 연결 드래그 상태
    bool is_connecting;
    int connecting_port_id;
    ImVec2 connecting_pos;

    // 뷰포트 상태
    ImVec2 scrolling;
    float zoom;

    // 드래깅 상태
    bool is_dragging_node;
    int dragging_node_id;
    ImVec2 drag_offset;

    // 패닝 상태
    bool is_panning;
    ImVec2 pan_start;

    // 그리드 설정
    float grid_size;

    // 그리드 스냅 설정
    bool enable_grid_snap;
    float snap_size;

    // 배경 설정
    ImVec2 background_size;
    bool use_background_image;
    ImTextureID background_texture;
    ImVec2 background_image_size;

public:
    Editor()
        : next_node_id(0), next_port_id(0), next_connection_id(0),
          scrolling(ImVec2(0, 0)), zoom(1.0f),
          is_dragging_node(false), dragging_node_id(-1),
          is_panning(false), is_connecting(false), connecting_port_id(-1),
          grid_size(64.0f), enable_grid_snap(true), snap_size(10.0f),
          background_size(ImVec2(0, 0)), use_background_image(false),
          background_texture(0), background_image_size(ImVec2(0, 0)) {}

    // 노드 추가
    void AddNode(const std::string& title, ImVec2 position) {
        auto node = std::make_shared<Node>(next_node_id++, position, title);
        nodes.push_back(node);
    }

    // 포트 추가
    int AddPort(int node_id, PortType type, const std::string& name) {
        auto port = std::make_shared<Port>(next_port_id++, node_id, type, name);
        ports.push_back(port);

        // 노드에 포트 ID 추가
        for (auto& node : nodes) {
            if (node->id == node_id) {
                if (type == PortType::Input) {
                    node->input_ports.push_back(port->id);
                } else {
                    node->output_ports.push_back(port->id);
                }
                break;
            }
        }

        return port->id;
    }

    // 연결 추가
    void AddConnection(int output_port_id, int input_port_id) {
        // 중복 연결 방지
        for (auto& conn : connections) {
            if (conn->output_port_id == output_port_id && conn->input_port_id == input_port_id) {
                return;
            }
        }

        auto connection = std::make_shared<Connection>(next_connection_id++, output_port_id, input_port_id);
        connections.push_back(connection);
    }

    // 배경 크기 설정
    void SetBackgroundSize(ImVec2 size) {
        background_size = size;
    }

    // 배경 이미지 설정
    void SetBackgroundImage(const char* filename) {
        ImTextureID texture;
        int width, height;
        if (LoadTextureFromFile(filename, &texture, &width, &height)) {
            background_texture = texture;
            background_image_size = ImVec2(width, height);
            background_size = background_image_size;
            use_background_image = (texture != 0);
        }
    }

    // 배경 이미지 제거
    void ClearBackgroundImage() {
        background_texture = 0;
        use_background_image = false;
    }

    // 배경 크기 가져오기
    ImVec2 GetBackgroundSize() const {
        return background_size;
    }

    // 그리드 스냅 활성화/비활성화
    void SetGridSnap(bool enable) {
        enable_grid_snap = enable;
    }

    // 스냅 크기 설정 (캔버스 단위 픽셀)
    void SetSnapSize(float size) {
        snap_size = size;
    }

    // 스냅 크기 가져오기
    float GetSnapSize() const {
        return snap_size;
    }

    // 그리드 스냅 활성화 여부
    bool IsGridSnapEnabled() const {
        return enable_grid_snap;
    }

    // 화면 좌표를 캔버스 좌표로 변환
    ImVec2 ScreenToCanvas(ImVec2 screen_pos, ImVec2 canvas_origin) {
        return ImVec2(
            (screen_pos.x - canvas_origin.x - scrolling.x) / zoom,
            (screen_pos.y - canvas_origin.y - scrolling.y) / zoom
        );
    }

    // 캔버스 좌표를 화면 좌표로 변환
    ImVec2 CanvasToScreen(ImVec2 canvas_pos, ImVec2 canvas_origin) {
        return ImVec2(
            canvas_pos.x * zoom + scrolling.x + canvas_origin.x,
            canvas_pos.y * zoom + scrolling.y + canvas_origin.y
        );
    }

    // 다단계 그리드 그리기
    void DrawGrid(ImDrawList* draw_list, ImVec2 canvas_origin, ImVec2 canvas_size) {
        // 다양한 그리드 레벨 정의 (캔버스 단위)
        float grid_levels[] = {1.0f, 5.0f, 10.0f, 50.0f, 100.0f, 500.0f, 1000.0f};
        ImU32 grid_colors[] = {
            IM_COL32(100, 100, 100, 80),  // 1px - 가장 세밀한 그리드
            IM_COL32(120, 120, 120, 100), // 5px
            IM_COL32(140, 140, 140, 120), // 10px
            IM_COL32(160, 160, 160, 140), // 50px
            IM_COL32(180, 180, 180, 160), // 100px
            IM_COL32(200, 200, 200, 180), // 500px
            IM_COL32(220, 220, 220, 200)  // 1000px
        };

        int num_levels = sizeof(grid_levels) / sizeof(grid_levels[0]);

        // 각 그리드 레벨을 그림
        for (int level = 0; level < num_levels; level++) {
            float grid_step = grid_levels[level] * zoom;

            // 그리드가 너무 작으면 건너뛰기 (4px 미만)
            if (grid_step < 4.0f) continue;

            // 그리드가 너무 크면 다음 레벨로
            if (grid_step > 200.0f && level < num_levels - 1) continue;

            // 페이드 효과: 그리드가 작아질수록 투명도 감소
            float alpha_factor = std::min(1.0f, (grid_step - 4.0f) / 20.0f);
            ImU32 grid_color = grid_colors[level];
            ImU32 faded_color = IM_COL32(
                (grid_color >> IM_COL32_R_SHIFT) & 0xFF,
                (grid_color >> IM_COL32_G_SHIFT) & 0xFF,
                (grid_color >> IM_COL32_B_SHIFT) & 0xFF,
                (int)(((grid_color >> IM_COL32_A_SHIFT) & 0xFF) * alpha_factor)
            );

            // 시작 오프셋 계산
            float x_offset = fmodf(scrolling.x, grid_step);
            float y_offset = fmodf(scrolling.y, grid_step);

            // 수직선 - 정수 픽셀 위치에 정렬
            for (float x = x_offset; x < canvas_size.x; x += grid_step) {
                float pixel_x = floorf(canvas_origin.x + x) + 0.5f; // 픽셀 중심에 정렬
                draw_list->AddLine(
                    ImVec2(pixel_x, canvas_origin.y),
                    ImVec2(pixel_x, canvas_origin.y + canvas_size.y),
                    faded_color
                );
            }

            // 수평선 - 정수 픽셀 위치에 정렬
            for (float y = y_offset; y < canvas_size.y; y += grid_step) {
                float pixel_y = floorf(canvas_origin.y + y) + 0.5f; // 픽셀 중심에 정렬
                draw_list->AddLine(
                    ImVec2(canvas_origin.x, pixel_y),
                    ImVec2(canvas_origin.x + canvas_size.x, pixel_y),
                    faded_color
                );
            }
        }
    }

    // 노드 그리기
    void DrawNode(ImDrawList* draw_list, std::shared_ptr<Node> node, ImVec2 canvas_origin) {
        ImVec2 screen_pos = CanvasToScreen(node->position, canvas_origin);
        ImVec2 scaled_size = ImVec2(node->size.x * zoom, node->size.y * zoom);

        // 노드 배경
        ImU32 bg_color = node->selected ?
            IM_COL32(120, 120, 180, 255) : node->color;
        draw_list->AddRectFilled(
            screen_pos,
            ImVec2(screen_pos.x + scaled_size.x, screen_pos.y + scaled_size.y),
            bg_color,
            4.0f * zoom
        );

        // 노드 테두리
        ImU32 border_color = node->selected ?
            IM_COL32(255, 255, 100, 255) : IM_COL32(60, 60, 80, 255);
        draw_list->AddRect(
            screen_pos,
            ImVec2(screen_pos.x + scaled_size.x, screen_pos.y + scaled_size.y),
            border_color,
            4.0f * zoom,
            0,
            2.0f
        );

        // 노드 타이틀 바
        draw_list->AddRectFilled(
            screen_pos,
            ImVec2(screen_pos.x + scaled_size.x, screen_pos.y + 25.0f * zoom),
            IM_COL32(80, 80, 120, 255),
            4.0f * zoom,
            ImDrawFlags_RoundCornersTop
        );

        // 타이틀 텍스트 (줌에 따라 폰트 크기 조절)
        if (zoom > 0.1f) {
            // 폰트 크기를 줌 레벨에 비례하도록 설정
            // 노드 크기도 zoom에 비례하므로 폰트도 zoom에 비례하게
            float font_scale = zoom;

            ImVec2 text_pos = ImVec2(
                screen_pos.x + 8.0f * zoom,
                screen_pos.y + 5.0f * zoom
            );

            // 폰트 크기 설정
            ImFont* font = ImGui::GetFont();
            float old_font_scale = font->Scale;
            font->Scale = font_scale;
            draw_list->PushTextureID(font->ContainerAtlas->TexID);

            // ImGui 폰트 크기는 font->FontSize가 아닌 ImGui::GetFontSize() 사용
            draw_list->AddText(font, ImGui::GetFontSize() * font_scale, text_pos,
                IM_COL32(255, 255, 255, 255), node->title.c_str());

            draw_list->PopTextureID();
            font->Scale = old_font_scale;
        }

        // 포트 그리기
        float port_radius = 6.0f * zoom;
        float port_spacing = 20.0f;

        // 입력 포트 (왼쪽)
        for (size_t i = 0; i < node->input_ports.size(); i++) {
            int port_id = node->input_ports[i];
            auto port = GetPort(port_id);
            if (port) {
                float y_pos = node->position.y + 35.0f + i * port_spacing;
                port->position = ImVec2(node->position.x, y_pos);

                ImVec2 port_screen_pos = CanvasToScreen(port->position, canvas_origin);
                draw_list->AddCircleFilled(port_screen_pos, port_radius, IM_COL32(100, 200, 100, 255));
                draw_list->AddCircle(port_screen_pos, port_radius, IM_COL32(50, 150, 50, 255), 0, 2.0f);
            }
        }

        // 출력 포트 (오른쪽)
        for (size_t i = 0; i < node->output_ports.size(); i++) {
            int port_id = node->output_ports[i];
            auto port = GetPort(port_id);
            if (port) {
                float y_pos = node->position.y + 35.0f + i * port_spacing;
                port->position = ImVec2(node->position.x + node->size.x, y_pos);

                ImVec2 port_screen_pos = CanvasToScreen(port->position, canvas_origin);
                draw_list->AddCircleFilled(port_screen_pos, port_radius, IM_COL32(200, 100, 100, 255));
                draw_list->AddCircle(port_screen_pos, port_radius, IM_COL32(150, 50, 50, 255), 0, 2.0f);
            }
        }
    }

    // 포트 가져오기
    std::shared_ptr<Port> GetPort(int port_id) {
        for (auto& port : ports) {
            if (port->id == port_id) {
                return port;
            }
        }
        return nullptr;
    }

    // 포트 히트 테스트
    std::shared_ptr<Port> GetPortAtPosition(ImVec2 canvas_pos, float radius = 10.0f) {
        for (auto& port : ports) {
            float dx = canvas_pos.x - port->position.x;
            float dy = canvas_pos.y - port->position.y;
            float dist = sqrtf(dx * dx + dy * dy);
            if (dist <= radius) {
                return port;
            }
        }
        return nullptr;
    }

    // 노드 히트 테스트
    std::shared_ptr<Node> GetNodeAtPosition(ImVec2 canvas_pos) {
        // 역순으로 검사 (위에 있는 노드 우선)
        for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
            auto& node = *it;
            if (canvas_pos.x >= node->position.x &&
                canvas_pos.x <= node->position.x + node->size.x &&
                canvas_pos.y >= node->position.y &&
                canvas_pos.y <= node->position.y + node->size.y) {
                return node;
            }
        }
        return nullptr;
    }

    // 베지어 곡선 그리기 (연결선용)
    void DrawBezierConnection(ImDrawList* draw_list, ImVec2 p1, ImVec2 p2, ImU32 color, float thickness = 3.0f) {
        float distance = std::abs(p2.x - p1.x);
        float offset = distance * 0.5f;

        ImVec2 cp1 = ImVec2(p1.x + offset, p1.y);
        ImVec2 cp2 = ImVec2(p2.x - offset, p2.y);

        draw_list->AddBezierCubic(p1, cp1, cp2, p2, color, thickness);
    }

    // 메인 렌더링 함수
    void Render() {
        ImVec2 canvas_origin = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();

        // 캔버스 영역 설정
        ImGui::InvisibleButton("canvas", canvas_size,
            ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight | ImGuiButtonFlags_MouseButtonMiddle);
        bool is_hovered = ImGui::IsItemHovered();
        bool is_active = ImGui::IsItemActive();

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        draw_list->PushClipRect(canvas_origin,
            ImVec2(canvas_origin.x + canvas_size.x, canvas_origin.y + canvas_size.y), true);

        // 배경 렌더링
        // 1. 전체 캔버스 영역 배경색
        draw_list->AddRectFilled(canvas_origin,
            ImVec2(canvas_origin.x + canvas_size.x, canvas_origin.y + canvas_size.y),
            IM_COL32(40, 40, 40, 255));

        // 2. 설정된 배경 크기 영역
        ImVec2 bg_min = CanvasToScreen(ImVec2(0, 0), canvas_origin);
        ImVec2 bg_max = CanvasToScreen(background_size, canvas_origin);

        // 배경 이미지 또는 단색 배경
        if (use_background_image && background_texture != 0) {
            draw_list->AddImage(
                background_texture,
                bg_min,
                bg_max,
                ImVec2(0, 0),
                ImVec2(1, 1),
                IM_COL32(255, 255, 255, 255)
            );
        } else {
            draw_list->AddRectFilled(bg_min, bg_max, IM_COL32(50, 50, 50, 255));
        }

        // 배경 경계선
        draw_list->AddRect(bg_min, bg_max, IM_COL32(100, 100, 100, 255), 0.0f, 0, 2.0f);

        // 그리드 그리기
        DrawGrid(draw_list, canvas_origin, canvas_size);

        // 마우스 입력 처리
        ImGuiIO& io = ImGui::GetIO();
        ImVec2 mouse_pos = io.MousePos;
        ImVec2 canvas_mouse_pos = ScreenToCanvas(mouse_pos, canvas_origin);

        // 줌 처리 (마우스 휠) - 마우스 위치를 중심으로 줌
        if (is_hovered && io.MouseWheel != 0.0f) {
            float old_zoom = zoom;

            // 줌 전 마우스의 캔버스 좌표 저장
            ImVec2 mouse_canvas_before = ScreenToCanvas(mouse_pos, canvas_origin);

            // 비례적 줌 델타 (15% 증가/감소)
            float zoom_factor = io.MouseWheel > 0 ? 1.15f : (1.0f / 1.15f);
            zoom = std::max(0.1f, std::min(100.0f, zoom * zoom_factor));

            // 줌 후 같은 화면 위치에서의 캔버스 좌표 계산
            ImVec2 mouse_canvas_after = ScreenToCanvas(mouse_pos, canvas_origin);

            // 스크롤 오프셋 조정하여 마우스 위치 고정
            scrolling.x += (mouse_canvas_after.x - mouse_canvas_before.x) * zoom;
            scrolling.y += (mouse_canvas_after.y - mouse_canvas_before.y) * zoom;
        }

        // 포트 클릭 및 연결 시작
        if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            auto clicked_port = GetPortAtPosition(canvas_mouse_pos);
            if (clicked_port) {
                // 포트 클릭 - 연결 시작
                is_connecting = true;
                connecting_port_id = clicked_port->id;
                connecting_pos = mouse_pos;
            } else {
                // 노드 드래깅 시작
                auto clicked_node = GetNodeAtPosition(canvas_mouse_pos);
                if (clicked_node) {
                    is_dragging_node = true;
                    dragging_node_id = clicked_node->id;
                    drag_offset = ImVec2(
                        canvas_mouse_pos.x - clicked_node->position.x,
                        canvas_mouse_pos.y - clicked_node->position.y
                    );

                    // 선택 상태 업데이트
                    if (!io.KeyCtrl) {
                        for (auto& node : nodes) {
                            node->selected = false;
                        }
                    }
                    clicked_node->selected = true;
                } else {
                    // 빈 공간 클릭 - 모든 선택 해제
                    if (!io.KeyCtrl) {
                        for (auto& node : nodes) {
                            node->selected = false;
                        }
                    }
                }
            }
        }

        // 연결 드래그 중
        if (is_connecting) {
            connecting_pos = mouse_pos;
        }

        // 연결 종료
        if (is_connecting && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            auto target_port = GetPortAtPosition(canvas_mouse_pos);
            if (target_port && target_port->id != connecting_port_id) {
                auto source_port = GetPort(connecting_port_id);
                if (source_port) {
                    // Output -> Input 연결만 허용
                    if (source_port->type == PortType::Output && target_port->type == PortType::Input) {
                        AddConnection(source_port->id, target_port->id);
                    } else if (source_port->type == PortType::Input && target_port->type == PortType::Output) {
                        AddConnection(target_port->id, source_port->id);
                    }
                }
            }
            is_connecting = false;
            connecting_port_id = -1;
        }

        // 노드 드래깅 중
        if (is_dragging_node && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            for (auto& node : nodes) {
                if (node->id == dragging_node_id) {
                    ImVec2 new_pos = ImVec2(
                        canvas_mouse_pos.x - drag_offset.x,
                        canvas_mouse_pos.y - drag_offset.y
                    );

                    // 그리드 스냅 적용
                    if (enable_grid_snap) {
                        new_pos.x = roundf(new_pos.x / snap_size) * snap_size;
                        new_pos.y = roundf(new_pos.y / snap_size) * snap_size;
                    }

                    node->position = new_pos;
                    break;
                }
            }
        }

        // 노드 드래깅 종료
        if (is_dragging_node && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            is_dragging_node = false;
            dragging_node_id = -1;
        }

        // 패닝 시작 (마우스 중간 버튼 또는 우클릭)
        if (is_hovered && (ImGui::IsMouseClicked(ImGuiMouseButton_Middle) ||
            (ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !GetNodeAtPosition(canvas_mouse_pos)))) {
            is_panning = true;
            pan_start = mouse_pos;
        }

        // 패닝 중
        if (is_panning && (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
            ImGui::IsMouseDragging(ImGuiMouseButton_Right))) {
            ImVec2 delta = ImVec2(
                mouse_pos.x - pan_start.x,
                mouse_pos.y - pan_start.y
            );
            scrolling.x += delta.x;
            scrolling.y += delta.y;
            pan_start = mouse_pos;
        }

        // 패닝 종료
        if (is_panning && (ImGui::IsMouseReleased(ImGuiMouseButton_Middle) ||
            ImGui::IsMouseReleased(ImGuiMouseButton_Right))) {
            is_panning = false;
        }

        // 연결선 그리기
        for (auto& conn : connections) {
            auto output_port = GetPort(conn->output_port_id);
            auto input_port = GetPort(conn->input_port_id);

            if (output_port && input_port) {
                ImVec2 p1 = CanvasToScreen(output_port->position, canvas_origin);
                ImVec2 p2 = CanvasToScreen(input_port->position, canvas_origin);
                DrawBezierConnection(draw_list, p1, p2, IM_COL32(200, 200, 200, 255), 3.0f * zoom);
            }
        }

        // 연결 드래그 중인 선 그리기
        if (is_connecting) {
            auto source_port = GetPort(connecting_port_id);
            if (source_port) {
                ImVec2 p1 = CanvasToScreen(source_port->position, canvas_origin);
                ImVec2 p2 = connecting_pos;
                DrawBezierConnection(draw_list, p1, p2, IM_COL32(255, 255, 100, 200), 3.0f * zoom);
            }
        }

        // 모든 노드 그리기
        for (auto& node : nodes) {
            DrawNode(draw_list, node, canvas_origin);
        }

        draw_list->PopClipRect();

        // 정보 표시
        ImGui::SetCursorScreenPos(ImVec2(canvas_origin.x + 10, canvas_origin.y + 10));
        ImGui::Text("줌: %.2fx (%.1f%%) | 노드: %d | 연결: %d", zoom, zoom * 100.0f, (int)nodes.size(), (int)connections.size());
        ImGui::Text("배경 크기: %.0fx%.0f | 배경 이미지: %s",
            background_size.x, background_size.y,
            use_background_image ? "사용" : "미사용");
        ImGui::Text("그리드 스냅: %s (%.0fpx)",
            enable_grid_snap ? "ON" : "OFF", snap_size);
        ImGui::Text("마우스 휠: 줌 | 중간/우클릭 드래그: 패닝");
        ImGui::Text("좌클릭 드래그: 노드 이동");
    }

    // 노드 개수 반환
    int GetNodeCount() const {
        return static_cast<int>(nodes.size());
    }
};

} // namespace NodeEditor

