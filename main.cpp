#include <iostream>
#include "lib/imgui/imgui_app.h"

int main()
{
    std::cout << "Hello, dev-imgui-app branch!" << std::endl;

    ImguiApp::start_background("Demo");

    while (ImguiApp::is_running())
    {
        ImguiApp::show_imgui([]()
        {

        });
    }

    ImguiApp::stop_background();


    return 0;
}