#include <implot.h>
#include <implot3d.h>
#include <iostream>
#include "imgui.h"








int main() {
    std::cout << "Hello, World!" << std::endl;




    ImGui::start("데모");

    while (ImGui::isRunning())
    {



        ImGui::context([&]()
        {
            ImGui::Begin("Draw");

            ImGui::End();

        });
    }
    ImGui::stop();


    return 0;
}
