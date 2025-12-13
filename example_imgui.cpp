#include "imgui.h"
#include <iostream>


int main()
{
    ImGui::start("데모");


    auto img = ImGui::load_image("../imgui/sample/doro1.png");


    while (ImGui::isRunning())
    {
        ImGui::context([&]()
                {
                    // ImGui 패널
                    ImGui::Begin("ImGui 패널");

                    ImGui::Text("버튼을 클릭하면 이미지가 바뀜니다.");

                    if (ImGui::Button(" 버튼 ")) {
                        img = ImGui::load_image("../imgui/sample/doro2.png");
                    }
                    ImGui::End(); // end ImGui 패널


                    // 이미지 렌더링
                    ImGui::Begin("이미지 렌더링");
                    if (img->texture_id)
                    {
                        ImGui::Image(img->texture_id, img->size);
                    }
                    ImGui::End(); // end 이미지 렌더링


                    // ImGui 데모
                    ImGui::ShowDemoWindow();
                });
    }


    ImGui::stop();

    return 0;
}