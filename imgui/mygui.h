#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>

#include "imgui_impl_opengl3_loader.h"

namespace ImGui
{
    void start(const char* title, const ImVec2& size = ImVec2(1280, 720));
    void stop();
    void context(std::function<void()> func);
    bool isRunning();

    struct Image_ {
        GLuint texture_id;
        ImVec2 size;

        Image_() : texture_id(0), size(0, 0) {}
        ~Image_();
    };
    void load_image(const std::string& path, GLuint& texture_id, int& width, int& height);
    std::shared_ptr<Image_> load_image(const std::string& path);
}
