#include "imgui.h"

int main()
{
    ImGui::start("데모");

    while (ImGui::isRunning())
    {
        ImGui::context([]()
        {
            ImGui::Begin("패널");
            ImGui::Text("Hello, ImGui!");
            ImGui::End();
        });
    }

    ImGui::stop();

    return 0;
}