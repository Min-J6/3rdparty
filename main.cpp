#include <iostream>
#include "imgui.h"


int main() {

    ImGui::start("데모");

    while (ImGui::isRunning())
    {
        ImGui::context([]()
                {
                    ImGui::Begin("창1");

                    ImGui::Text("abc123");
                    ImGui::Text("ABC456");

                    ImGui::End();
                });
    }

    ImGui::stop();

    std::cout << "Hello World!" << std::endl;

    return 0;
}