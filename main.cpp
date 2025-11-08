#include <iostream>

#include "imgui.h"

int main() {
    std::cout << "Hello, World!" << std::endl;


    ImGui::start("데모");

    while (ImGui::isRunning())
    {
        ImGui::draw([]()
        {
            ImGui::Begin("dasd");

            ImGui::End();

        });
    }

    ImGui::stop();
    return 0;
}
